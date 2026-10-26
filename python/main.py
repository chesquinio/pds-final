import threading
import time
import numpy as np
from datetime import datetime
from collections import deque
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI

web_ui = WebUI()

# --- PAR�METROS DSP Y DE MUESTREO NATIVO A 1000 HZ ---
FS = 500.0  # Frecuencia de muestreo optimizada (Hz)
DT = 1.0 / FS
TAMANO_FFT = 512

buffer_fft = deque(maxlen=TAMANO_FFT)

muestras_batch = []
batch_lock = threading.Lock()

def recibir_muestra(vin_mv: int, vout_mv: int):
    v_in = float(vin_mv) / 1000.0
    v_out = float(vout_mv) / 1000.0
    buffer_fft.append(v_in)
    with batch_lock:
        muestras_batch.append({'v_in': round(v_in, 3), 'v_out': round(v_out, 3)})

Bridge.provide("m", recibir_muestra)

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
    Calcula la FFT con ventana de Hanning, interpolacion parabolica y deteccion
    de las 3 primeras arm�nicas (fundamental, 2 y 3) en el rango completo (hasta 120 Hz).
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
    
    # Busqueda de Fundamental entre 1.0 Hz y 248.0 Hz (dentro de Nyquist = 250 Hz)
    idx_min = max(1, int(1.0 / df))
    idx_max = min(len(magnitudes) - 2, int(248.0 / df))
    
    if idx_max <= idx_min:
        return None
        
    k1 = idx_min + int(np.argmax(magnitudes[idx_min:idx_max]))
    amp1_raw = float(magnitudes[k1])
    
    # Umbral de deteccion minima (40 mV)
    if amp1_raw < 0.04:
        return {
            'armonicas': [
                {'armonica': 1, 'nombre': 'Fundamental', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 100.0},
                {'armonica': 2, 'nombre': '2� Arm�nica', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 0.0},
                {'armonica': 3, 'nombre': '3� Arm�nica', 'frec': 0.0, 'amp': 0.0, 'vpp': 0.0, 'pct': 0.0}
            ],
            'thd': 0.0,
            'v_dc': round(media_dc, 2),
            'v_rms': round(float(np.std(senal)), 2),
            'v_pp': round(float(np.ptp(senal)), 2),
            'espectro': []
        }
        
    # Interpolacion cuadratica para afinar frecuencia del pico
    y1, y2, y3 = float(magnitudes[k1 - 1]), float(magnitudes[k1]), float(magnitudes[k1 + 1])
    denom = y1 - 2.0 * y2 + y3
    delta = 0.5 * (y1 - y3) / denom if abs(denom) > 1e-9 else 0.0
    f1 = (k1 + delta) * df
    
    # Correccion de festoneado de Hanning
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
    
    # Deteccion de 2 y 3 armonica
    k_teorico_prev = k1
    for mult in [2, 3]:
        f_teorica = mult * f1
        k_teorico = int(round(f_teorica / df))
        
        # Debe encontrarse por debajo del limite de Nyquist (FS/2 = 250 Hz)
        if k_teorico + 1 < len(magnitudes):
            # Proteger contra invasion del lobulo principal de la armonica previa
            k_ini = max(k_teorico_prev + 2, k_teorico - 2)
            k_fin = min(len(magnitudes) - 2, k_teorico + 2)
            k_teorico_prev = k_teorico
            
            if k_fin >= k_ini:
                k_h = k_ini + int(np.argmax(magnitudes[k_ini:k_fin + 1]))
                
                # Umbral de armonica: por encima del piso de ruido (~30 mV o 2.5% de la fundamental)
                umbral_armonica = max(0.03, 0.025 * a1)
                if magnitudes[k_h] >= umbral_armonica:
                    # Ajuste fino si es pico local
                    if magnitudes[k_h] >= magnitudes[k_h - 1] and magnitudes[k_h] >= magnitudes[k_h + 1]:
                        yh1, yh2, yh3 = float(magnitudes[k_h - 1]), float(magnitudes[k_h]), float(magnitudes[k_h + 1])
                        denom_h = yh1 - 2.0 * yh2 + yh3
                        delta_h = 0.5 * (yh1 - yh3) / denom_h if abs(denom_h) > 1e-9 else 0.0
                        fh = (k_h + delta_h) * df
                        val_sinc_h = sinc(delta_h) / (1.0 - delta_h**2) if abs(delta_h) < 0.999 else 1.0
                        corr_h = 1.0 / abs(val_sinc_h) if abs(val_sinc_h) > 1e-4 else 1.0
                        ah = yh2 * corr_h
                    else:
                        fh = f_teorica
                        ah = float(magnitudes[k_h])
                    
                    resultado.append({
                        'armonica': mult,
                        'nombre': f'{mult}ª Armónica',
                        'frec': round(float(fh), 1),
                        'amp': round(float(ah), 2),
                        'vpp': round(float(ah * 2.0), 2),
                        'pct': round(float((ah / a1) * 100.0), 1),
                        'estado': 'ok'
                    })
                else:
                    resultado.append({
                        'armonica': mult,
                        'nombre': f'{mult}ª Armónica',
                        'frec': round(float(f_teorica), 1),
                        'amp': 0.0,
                        'vpp': 0.0,
                        'pct': 0.0,
                        'estado': 'no_detectada'
                    })
            else:
                resultado.append({
                    'armonica': mult,
                    'nombre': f'{mult}ª Armónica',
                    'frec': round(float(f_teorica), 1),
                    'amp': 0.0,
                    'vpp': 0.0,
                    'pct': 0.0,
                    'estado': 'no_detectada'
                })
        else:
            resultado.append({
                'armonica': mult,
                'nombre': f'{mult}ª Armónica',
                'frec': round(float(f_teorica), 1),
                'amp': 0.0,
                'vpp': 0.0,
                'pct': 0.0,
                'estado': 'fuera_nyquist'
            })
            
    a2 = resultado[1]['amp']
    a3 = resultado[2]['amp']
    thd = (np.sqrt(a2**2 + a3**2) / a1) * 100.0 if a1 > 0.05 else 0.0
    
    # Mini-espectro para graficar en pantalla (hasta 250 Hz en ~80 puntos nítidos)
    max_k_esp = min(len(magnitudes), int(250.0 / df))
    paso_esp = max(1, max_k_esp // 80)
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
                Bridge.call('configurarFiltro', cfg['tipo'], float(cfg['fc']))
            except Exception as e:
                print(f"Error al enviar configurarFiltro: {e}")
                
        # 2. Enviar comando de control de muestreo pendiente (Consigna 2)
        if comando_muestreo is not None:
            cmd = comando_muestreo
            comando_muestreo = None
            try:
                Bridge.call('controlMuestreo', cmd)
            except Exception as e:
                print(f"Error al enviar controlMuestreo: {e}")

        # 3. Extraer muestras recién llegadas directamente por streaming asíncrono
        with batch_lock:
            if muestras_batch:
                lote = list(muestras_batch)
                muestras_batch.clear()
            else:
                lote = []
        
        if lote:
            # Calculo de la FFT en ventana deslizante cada ~100 ms (5 ciclos a ~50 FPS)
            contador_fft += 1
            info_fft = None
            if contador_fft >= 5:
                contador_fft = 0
                if len(buffer_fft) >= TAMANO_FFT:
                    info_fft = procesar_fft(list(buffer_fft))
            
            paquete = {
                'muestras': lote,
                'muestreo_activo': muestreo_activo,
                'fs': FS
            }
            if info_fft:
                paquete['fft_data'] = info_fft
                
            web_ui.send_message('datos_dsp', paquete)
            
    except Exception as e:
        print(f"Error en bucle_control_senal: {e}")
        
    time.sleep(0.02) # ~50 cuadros por segundo para streaming continuo y fluido

if __name__ == '__main__':
    App.run(user_loop=bucle_control_senal)
