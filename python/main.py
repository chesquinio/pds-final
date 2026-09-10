import threading
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
        self._lock = threading.Lock()
        self._sock = None

    def _conectar(self):
        try:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.3)
            s.connect(self.socket_path)
            self._sock = s
            return self._sock
        except Exception as e:
            self._sock = None
            return None

    def call(self, method, *args):
        with self._lock:
            self.msgid = (self.msgid + 1) & 0xFFFFFFFF
            current_id = self.msgid
            req = [0, current_id, method, list(args)]
            payload = msgpack.packb(req, use_bin_type=True)
            
            for intento in range(2):
                if not self._sock:
                    if not self._conectar():
                        time.sleep(0.01)
                        continue
                try:
                    self._sock.sendall(payload)
                    unpacker = msgpack.Unpacker(raw=False)
                    while True:
                        data = self._sock.recv(4096)
                        if not data:
                            raise ConnectionResetError("Socket cerrado por el bridge")
                        unpacker.feed(data)
                        for msg in unpacker:
                            if len(msg) == 4 and msg[0] == 1 and msg[1] == current_id:
                                return msg[3]
                except Exception as e:
                    # En caso de error o desconexión, cerramos y reintentamos una vez
                    self._conectar()
            return None

rpc = BridgeSocketClient("/var/run/arduino-router.sock")
web_ui = WebUI()

# --- PARÁMETROS DSP Y DE MUESTREO NATIVO A 1000 HZ ---
FS = 1000.0  # Frecuencia de muestreo del hardware (Hz)
DT = 1.0 / FS
TAMANO_FFT = 1024  # 1.024 segundos de señal -> Delta_f = 0.976 Hz

buffer_fft = deque(maxlen=TAMANO_FFT)

# Estado y sincronización de controles
config_filtro = {'tipo': 'ninguno', 'fc': 20.0}
nueva_configuracion = None
comando_muestreo = None
muestreo_activo = True

def recibir_configuracion(sid, data):
    """ Escucha cambios de filtro desde la interfaz web """
    global nueva_configuracion, config_filtro
    tipo_filtro = data.get('tipo', 'ninguno')
    fc = float(data.get('fc', 20.0))
    config_filtro['tipo'] = tipo_filtro
    config_filtro['fc'] = fc

    mapa_filtros = {'ninguno': 0, 'pasabajas': 1, 'pasaaltos': 2, 'pasabanda': 3}
    nueva_configuracion = {'tipo': mapa_filtros.get(tipo_filtro, 0), 'fc': fc}

def recibir_control_muestreo(sid, data):
    """ Permite iniciar o detener el muestreo de adquisición (Consigna 2) """
    global comando_muestreo, muestreo_activo
    activo = bool(data.get('activo', True))
    muestreo_activo = activo
    comando_muestreo = 1 if activo else 0

web_ui.on_message('actualizar_filtro', recibir_configuracion)
web_ui.on_message('control_muestreo', recibir_control_muestreo)

def sinc(x):
    if abs(x) < 1e-6:
        return 1.0
    return np.sin(np.pi * x) / (np.pi * x)

def procesar_fft(datos):
    """
    Calcula la FFT con ventana de Hanning, interpolación parabólica y detección
    de las 3 primeras armónicas (fundamental, 2ª y 3ª) en el rango completo (hasta 120 Hz).
    """
    if len(datos) < TAMANO_FFT:
        return None
        
    senal = np.array(datos, dtype=np.float64)
    media_dc = float(np.mean(senal))
    senal_ac = senal - media_dc
    
    ventana = np.hanning(TAMANO_FFT)
    ganancia_coherente = float(np.sum(ventana))
    senal_ventaneada = senal_ac * ventana
    
    # RFFT: amplitudes normalizadas en Vp
    espectro = np.fft.rfft(senal_ventaneada)
    magnitudes = np.abs(espectro) * (2.0 / ganancia_coherente)
    df = FS / TAMANO_FFT
    
    # Búsqueda de Fundamental entre 1.0 Hz y 125.0 Hz
    idx_min = max(1, int(1.0 / df))
    idx_max = min(len(magnitudes) - 2, int(125.0 / df))
    
    if idx_max <= idx_min:
        return None
        
    k1 = idx_min + int(np.argmax(magnitudes[idx_min:idx_max]))
    amp1_raw = float(magnitudes[k1])
    
    # Umbral de detección mínima (40 mV)
    if amp1_raw < 0.04:
        return {
            'armonicas': [
                {'armonica': 1, 'nombre': 'Fundamental', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 100.0},
                {'armonica': 2, 'nombre': '2ª Armónica', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 0.0},
                {'armonica': 3, 'nombre': '3ª Armónica', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 0.0}
            ],
            'thd': 0.0,
            'v_dc': round(media_dc, 2),
            'v_rms': round(float(np.std(senal)), 2),
            'v_pp': round(float(np.ptp(senal)), 2),
            'espectro': []
        }
        
    # Interpolación cuadrática para afinar frecuencia del pico
    y1, y2, y3 = float(magnitudes[k1 - 1]), float(magnitudes[k1]), float(magnitudes[k1 + 1])
    denom = y1 - 2.0 * y2 + y3
    delta = 0.5 * (y1 - y3) / denom if abs(denom) > 1e-9 else 0.0
    f1 = (k1 + delta) * df
    
    # Corrección de festoneado de Hanning
    val_sinc = sinc(delta) / (1.0 - delta**2) if abs(delta) < 0.999 else 1.0
    corr_amp = 1.0 / abs(val_sinc) if abs(val_sinc) > 1e-4 else 1.0
    a1 = y2 * corr_amp
    
    resultado = [{
        'armonica': 1,
        'nombre': 'Fundamental',
        'frec': round(float(f1), 1),
        'amp': round(float(a1), 2),
        'vpp': round(float(a1 * 2.0), 2),
        'pct': 100.0
    }]
    
    # Detección de 2ª y 3ª armónica
    for mult in [2, 3]:
        f_teorica = mult * f1
        k_teorico = int(round(f_teorica / df))
        
        # Debe encontrarse por debajo del límite de Nyquist (FS/2 = 500 Hz)
        if k_teorico + 2 < len(magnitudes):
            k_ini = max(1, k_teorico - 3)
            k_fin = min(len(magnitudes) - 2, k_teorico + 4)
            k_h = k_ini + int(np.argmax(magnitudes[k_ini:k_fin]))
            
            umbral_armonica = max(0.04, 0.015 * a1)
            if (magnitudes[k_h] > magnitudes[k_h - 1] and 
                magnitudes[k_h] > magnitudes[k_h + 1] and 
                magnitudes[k_h] >= umbral_armonica):
                
                yh1, yh2, yh3 = float(magnitudes[k_h - 1]), float(magnitudes[k_h]), float(magnitudes[k_h + 1])
                denom_h = yh1 - 2.0 * yh2 + yh3
                delta_h = 0.5 * (yh1 - yh3) / denom_h if abs(denom_h) > 1e-9 else 0.0
                fh = (k_h + delta_h) * df
                
                val_sinc_h = sinc(delta_h) / (1.0 - delta_h**2) if abs(delta_h) < 0.999 else 1.0
                corr_h = 1.0 / abs(val_sinc_h) if abs(val_sinc_h) > 1e-4 else 1.0
                ah = yh2 * corr_h
                
                resultado.append({
                    'armonica': mult,
                    'nombre': f'{mult}ª Armónica',
                    'frec': round(float(fh), 1),
                    'amp': round(float(ah), 2),
                    'vpp': round(float(ah * 2.0), 2),
                    'pct': round(float((ah / a1) * 100.0), 1)
                })
            else:
                resultado.append({
                    'armonica': mult,
                    'nombre': f'{mult}ª Armónica',
                    'frec': round(float(f_teorica), 1),
                    'amp': 0.0,
                    'vpp': 0.0,
                    'pct': 0.0
                })
        else:
            resultado.append({
                'armonica': mult,
                'nombre': f'{mult}ª Armónica',
                'frec': round(float(f_teorica), 1),
                'amp': 0.0,
                'vpp': 0.0,
                'pct': 0.0
            })
            
    a2 = resultado[1]['amp']
    a3 = resultado[2]['amp']
    thd = (np.sqrt(a2**2 + a3**2) / a1) * 100.0 if a1 > 0.05 else 0.0
    
    # Mini-espectro para graficar en pantalla (hasta 350 Hz en 60 puntos)
    max_k_esp = min(len(magnitudes), int(350.0 / df))
    paso_esp = max(1, max_k_esp // 60)
    mini_espectro = []
    for bi in range(0, max_k_esp, paso_esp):
        mini_espectro.append({
            'frec': round(float(bi * df), 1),
            'amp': round(float(np.max(magnitudes[bi:bi+paso_esp])), 2)
        })
        
    return {
        'armonicas': resultado,
        'thd': round(float(thd), 2),
        'v_dc': round(media_dc, 2),
        'v_rms': round(float(np.std(senal)), 2),
        'v_pp': round(float(np.ptp(senal)), 2),
        'espectro': mini_espectro
    }

contador_fft = 0

def bucle_control_senal():
    global contador_fft, nueva_configuracion, comando_muestreo, muestreo_activo
    
    try:
        # 1. Enviar cambios de filtro pendientes hacia el MCU
        if nueva_configuracion is not None:
            cfg = nueva_configuracion
            nueva_configuracion = None
            try:
                rpc.call('configurarFiltro', cfg['tipo'], float(cfg['fc']))
            except Exception as e:
                print(f"Error al enviar configurarFiltro: {e}")
                
        # 2. Enviar comando de control de muestreo pendiente (Consigna 2)
        if comando_muestreo is not None:
            cmd = comando_muestreo
            comando_muestreo = None
            try:
                rpc.call('controlMuestreo', cmd)
            except Exception as e:
                print(f"Error al enviar controlMuestreo: {e}")

        # 3. Leer lote de muestras recopiladas en el MCU a 1000 Hz
        # El MCU devuelve valores int16 en milivoltios: [vin_mv_0, vout_mv_0, ...]
        datos_mcu = rpc.call('leerTelemetria')
        
        if datos_mcu and len(datos_mcu) >= 2:
            muestras_batch = []
            tiempo_base = datetime.now()
            
            for i in range(0, len(datos_mcu), 2):
                v_in = float(datos_mcu[i]) / 1000.0
                v_out = float(datos_mcu[i + 1]) / 1000.0
                
                buffer_fft.append(v_in)
                muestras_batch.append({
                    'v_in': round(v_in, 3),
                    'v_out': round(v_out, 3),
                    'tiempo': tiempo_base.strftime('%H:%M:%S.%f')[:-3]
                })
            
            # Cálculo de la FFT aproximadamente cada 200 ms (5 ciclos a ~25 FPS)
            contador_fft += 1
            info_fft = None
            if contador_fft >= 5:
                contador_fft = 0
                info_fft = procesar_fft(list(buffer_fft))
            
            paquete = {
                'muestras': muestras_batch,
                'muestreo_activo': muestreo_activo,
                'fs': FS
            }
            if info_fft:
                paquete['fft_data'] = info_fft
                
            web_ui.send_message('datos_dsp', paquete)
            
    except Exception as e:
        print(f"Error en bucle_control_senal: {e}")
        
    time.sleep(0.04) # ~25 cuadros por segundo hacia la UI web

if __name__ == '__main__':
    App.run(user_loop=bucle_control_senal)
