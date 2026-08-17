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

# --- CONFIGURACIÓN DE MUESTREO ---
FS = 100.0  # Frecuencia de muestreo (Hz)
DT = 1.0 / FS
TAMANO_FFT = 256  # Muestras para la FFT (~2.5 segundos)

buffer_fft = deque(maxlen=TAMANO_FFT)

# --- ESTADO DE LOS FILTROS DIGITALES ---
config_filtro = {'tipo': 'ninguno', 'fc': 5.0} # Valores por defecto

# Memorias para las ecuaciones de diferencias (Filtros IIR)
x_prev = 0.0
y_lp_prev = 0.0
y_hp_prev = 0.0
y_bp_lp_prev = 0.0
y_bp_hp_prev = 0.0

def recibir_configuracion(sid, data):
    """ Escucha los cambios que el usuario hace en la página web """
    global config_filtro
    config_filtro['tipo'] = data.get('tipo', 'ninguno')
    config_filtro['fc'] = float(data.get('fc', 5.0))

# Registramos el evento de red
web_ui.on_message('actualizar_filtro', recibir_configuracion)

def procesar_fft(datos):
    """ Calcula la FFT y extrae las 3 primeras armónicas """
    if len(datos) < TAMANO_FFT:
        return [(0,0), (0,0), (0,0)]
    
    # Quitamos el nivel de continua (Offset) para no opacar las armónicas
    senal_centrada = np.array(datos) - np.mean(datos)
    
    # Aplicamos FFT
    magnitudes = np.abs(np.fft.rfft(senal_centrada)) * (2.0 / TAMANO_FFT)
    frecuencias = np.fft.rfftfreq(TAMANO_FFT, d=DT)
    
    # Búsqueda de picos locales (Armónicas)
    picos = []
    for i in range(1, len(magnitudes)-1):
        # Si es mayor que sus vecinos, es un pico
        if magnitudes[i] > magnitudes[i-1] and magnitudes[i] > magnitudes[i+1]:
            if frecuencias[i] > 0.5: # Ignoramos ruido de muy baja frecuencia
                picos.append({'frec': float(frecuencias[i]), 'amp': float(magnitudes[i])})
                
    # Ordenamos por amplitud de mayor a menor y sacamos las 3 mejores
    picos.sort(key=lambda x: x['amp'], reverse=True)
    top_3 = picos[:3]
    
    # Rellenamos con ceros si hay menos de 3 picos detectables
    while len(top_3) < 3:
        top_3.append({'frec': 0.0, 'amp': 0.0})
        
    return top_3

def aplicar_filtro(x_actual, tipo, fc):
    """ Matemáticas de Filtros Digitales IIR (Primer Orden) """
    global x_prev, y_lp_prev, y_hp_prev, y_bp_lp_prev, y_bp_hp_prev
    
    # Si la frecuencia de corte es 0, evitamos división por cero
    if fc <= 0.1: fc = 0.1 
    
    # Constante de tiempo RC
    RC = 1.0 / (2.0 * np.pi * fc)
    
    salida = x_actual
    
    if tipo == 'pasabajas':
        # Ecuación LPF: y[n] = alfa * x[n] + (1-alfa) * y[n-1]
        alfa_lp = DT / (RC + DT)
        salida = (alfa_lp * x_actual) + ((1.0 - alfa_lp) * y_lp_prev)
        y_lp_prev = salida
        
    elif tipo == 'pasaaltos':
        # Ecuación HPF: y[n] = alfa * (y[n-1] + x[n] - x[n-1])
        alfa_hp = RC / (RC + DT)
        salida = alfa_hp * (y_hp_prev + x_actual - x_prev)
        y_hp_prev = salida
        
    elif tipo == 'pasabanda':
        # Cascada: Primero un Pasabajas (Cortamos altas freq)
        fc_alta = fc + 2.0  # Ancho de banda fijo de +/- 2Hz para probar
        RC_lp = 1.0 / (2.0 * np.pi * fc_alta)
        alfa_lp = DT / (RC_lp + DT)
        y_lp = (alfa_lp * x_actual) + ((1.0 - alfa_lp) * y_bp_lp_prev)
        y_bp_lp_prev = y_lp
        
        # Luego la salida del pasabajas entra a un Pasaaltos (Cortamos bajas freq)
        fc_baja = max(0.1, fc - 2.0)
        RC_hp = 1.0 / (2.0 * np.pi * fc_baja)
        alfa_hp = RC_hp / (RC_hp + DT)
        salida = alfa_hp * (y_bp_hp_prev + y_lp - x_prev)
        y_bp_hp_prev = salida

    x_prev = x_actual # Guardar muestra para el próximo ciclo
    return salida

contador_fft = 0

def bucle_control_senal():
    global contador_fft
    
    try:
        valor_adc = rpc.call('leerA4')
        
        if valor_adc is not None:
            # 1. Level Shifter (Reconstrucción -6V a +6V)
            voltaje_arduino = (valor_adc / 4095.0) * 3.3
            voltaje_real_in = (voltaje_arduino - 1.65) * 3.6363
            
            # 2. FILTRADO DIGITAL (Consigna 4)
            voltaje_real_out = aplicar_filtro(voltaje_real_in, config_filtro['tipo'], config_filtro['fc'])
            
            # (Futura implementación DAC) - Aquí convertirías voltaje_real_out de vuelta a 0-255
            
            # 3. FFT (Consigna 3)
            buffer_fft.append(voltaje_real_in)
            contador_fft += 1
            armonicas = []
            # Calculamos la FFT cada 10 ciclos para no saturar el procesador
            if contador_fft >= 10: 
                armonicas = procesar_fft(list(buffer_fft))
                contador_fft = 0
            
            # 4. Enviar datos a la UI
            tiempo_exacto = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            paquete = {
                'v_in': voltaje_real_in,
                'v_out': voltaje_real_out,
                'tiempo': tiempo_exacto
            }
            if armonicas:
                paquete['fft'] = armonicas
                
            web_ui.send_message('datos_dsp', paquete)
            
    except Exception as e:
        print(f"Error en loop: {e}")
        
    time.sleep(DT) # Limitado a 100 Hz

if __name__ == '__main__':
    App.run(user_loop=bucle_control_senal)