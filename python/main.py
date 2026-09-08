import time
import socket
import msgpack
import numpy as np
from datetime import datetime
from collections import deque
from arduino.app_utils import App
from arduino.app_bricks.web_ui import WebUI

class BridgeSocketClient:
    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.msgid = 0
    def call(self, method, *args):
        self.msgid += 1
        req = [0, self.msgid, method, list(args)]
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(self.socket_path)
            sock.sendall(msgpack.packb(req, use_bin_type=True))
            unpacker = msgpack.Unpacker(raw=False)
            while True:
                data = sock.recv(4096)
                if not data: raise ConnectionError("Cerrado")
                unpacker.feed(data)
                for msg in unpacker:
                    if len(msg) == 4 and msg[0] == 1 and msg[1] == self.msgid:
                        return msg[3]
        return None

rpc = BridgeSocketClient("/var/run/arduino-router.sock")
web_ui = WebUI()

# --- CONFIGURACIÓN DE MUESTREO (TELEMETRÍA Y FFT) ---
FS = 200.0  # Frecuencia de muestreo de telemetría (Hz)
DT = 1.0 / FS
TAMANO_FFT = 256  # Muestras para la FFT (1.28 segundos de señal)

buffer_fft = deque(maxlen=TAMANO_FFT)

# --- ESTADO DE LOS FILTROS DIGITALES ---
config_filtro = {'tipo': 'ninguno', 'fc': 5.0}

def recibir_configuracion(sid, data):
    """ Escucha los cambios que el usuario hace en la página web y los envía al MCU """
    global config_filtro
    
    tipo_filtro = data.get('tipo', 'ninguno')
    fc = float(data.get('fc', 5.0))
    config_filtro['tipo'] = tipo_filtro
    config_filtro['fc'] = fc

    mapa_filtros = {
        'ninguno': 0,
        'pasabajas': 1,
        'pasaaltos': 2,
        'pasabanda': 3
    }
    id_filtro = mapa_filtros.get(tipo_filtro, 0)
    
    try:
        rpc.call('configurarFiltro', id_filtro, fc)
    except Exception as e:
        print(f"Error al enviar configurarFiltro al Arduino: {e}")

web_ui.on_message('actualizar_filtro', recibir_configuracion)

def sinc(x):
    if abs(x) < 1e-6:
        return 1.0
    return np.sin(np.pi * x) / (np.pi * x)

def procesar_fft(datos):
    """
    Calcula el espectro con ventana de Hanning, corrección de ganancia coherente,
    interpolación parabólica y búsqueda específica de armónicas (fundamental, 2ª y 3ª).
    """
    if len(datos) < TAMANO_FFT:
        return [{'frec': 0.0, 'amp': 0.0}] * 3
    
    senal = np.array(datos)
    # 1. Eliminar componente continua (DC)
    senal_centrada = senal - np.mean(senal)
    
    # 2. Ventana Hanning y compensación de ganancia coherente
    ventana = np.hanning(TAMANO_FFT)
    ganancia_coherente = np.sum(ventana)
    senal_ventaneada = senal_centrada * ventana
    
    # 3. FFT de un solo lado (rfft)
    magnitudes = np.abs(np.fft.rfft(senal_ventaneada)) * (2.0 / ganancia_coherente)
    df = FS / TAMANO_FFT
    
    # 4. Búsqueda de la Fundamental (entre 0.5 Hz y 35 Hz)
    idx_min = max(1, int(0.5 / df))
    idx_max = min(len(magnitudes) - 2, int(35.0 / df))
    
    if idx_max <= idx_min:
        return [{'frec': 0.0, 'amp': 0.0}] * 3
        
    k1 = idx_min + int(np.argmax(magnitudes[idx_min:idx_max]))
    amp1_raw = magnitudes[k1]
    
    # Umbral de corte de ruido (80 mV)
    if amp1_raw < 0.08:
        return [{'frec': 0.0, 'amp': 0.0}] * 3
        
    # Interpolación cuadrática para afinar frecuencia del pico
    y1, y2, y3 = magnitudes[k1 - 1], magnitudes[k1], magnitudes[k1 + 1]
    denom = y1 - 2.0 * y2 + y3
    delta = 0.5 * (y1 - y3) / denom if denom != 0 else 0.0
    f1 = (k1 + delta) * df
    
    # Corrección de festoneado (scalloping loss) de la ventana Hanning
    val_sinc = sinc(delta) / (1.0 - delta**2) if abs(delta) < 1.0 else 1.0
    corr_amp = 1.0 / val_sinc if val_sinc != 0 else 1.0
    a1 = y2 * corr_amp
    
    resultado = [{'frec': round(float(f1), 1), 'amp': round(float(a1), 2)}]
    
    # 5. Detección específica de 2ª y 3ª armónica (en los múltiplos 2*f1 y 3*f1)
    for mult in [2, 3]:
        f_teorica = mult * f1
        k_teorico = int(round(f_teorica / df))
        
        # Debe estar dentro del límite de Nyquist
        if k_teorico + 2 < len(magnitudes):
            # Ventana reducida de búsqueda (±2 bins alrededor del múltiplo)
            k_ini = max(1, k_teorico - 2)
            k_fin = min(len(magnitudes) - 2, k_teorico + 3)
            k_h = k_ini + int(np.argmax(magnitudes[k_ini:k_fin]))
            
            umbral_armonica = max(0.08, 0.03 * a1) # Al menos 80mV o 3% de la fundamental
            # Debe ser un pico local relativo a sus vecinos contiguos y superar el umbral
            if (magnitudes[k_h] > magnitudes[k_h - 1] and 
                magnitudes[k_h] > magnitudes[k_h + 1] and 
                magnitudes[k_h] >= umbral_armonica):
                
                yh1, yh2, yh3 = magnitudes[k_h - 1], magnitudes[k_h], magnitudes[k_h + 1]
                denom_h = yh1 - 2.0 * yh2 + yh3
                delta_h = 0.5 * (yh1 - yh3) / denom_h if denom_h != 0 else 0.0
                fh = (k_h + delta_h) * df
                
                val_sinc_h = sinc(delta_h) / (1.0 - delta_h**2) if abs(delta_h) < 1.0 else 1.0
                corr_h = 1.0 / val_sinc_h if val_sinc_h != 0 else 1.0
                ah = yh2 * corr_h
                
                resultado.append({'frec': round(float(fh), 1), 'amp': round(float(ah), 2)})
            else:
                # No hay componente armónica medible en este múltiplo
                resultado.append({'frec': round(float(f_teorica), 1), 'amp': 0.0})
        else:
            resultado.append({'frec': 0.0, 'amp': 0.0})
            
    return resultado

contador_fft = 0

def bucle_control_senal():
    global contador_fft
    
    try:
        # Leemos el lote de muestras recopiladas por el microcontrolador a 200 Hz
        datos_mcu = rpc.call('leerTelemetria')
        
        if datos_mcu and len(datos_mcu) >= 2:
            muestras_batch = []
            tiempo_base = datetime.now()
            
            # datos_mcu contiene pares planos: [vin_0, vout_0, vin_1, vout_1, ...]
            for i in range(0, len(datos_mcu), 2):
                v_in = float(datos_mcu[i])
                v_out = float(datos_mcu[i + 1])
                
                buffer_fft.append(v_in)
                muestras_batch.append({
                    'v_in': round(v_in, 3),
                    'v_out': round(v_out, 3),
                    'tiempo': tiempo_base.strftime('%H:%M:%S.%f')[:-3]
                })
            
            contador_fft += 1
            armonicas = []
            # Calculamos la FFT aproximadamente cada ~240 ms
            if contador_fft >= 6:
                armonicas = procesar_fft(list(buffer_fft))
                contador_fft = 0
            
            paquete = {'muestras': muestras_batch}
            if armonicas:
                paquete['fft'] = armonicas
                
            web_ui.send_message('datos_dsp', paquete)
            
    except Exception as e:
        print(f"Error en loop: {e}")
        
    time.sleep(0.04) # ~25 cuadros por segundo para la interfaz web

if __name__ == '__main__':
    App.run(user_loop=bucle_control_senal)