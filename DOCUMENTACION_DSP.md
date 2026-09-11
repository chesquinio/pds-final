# DOCUMENTACIÓN TÉCNICA Y GUÍA DE DEFENSA
## PROCESAMIENTO DIGITAL DE SEÑALES (PDS) - ARDUINO UNO Q

---

## 1. RESUMEN Y ARQUITECTURA DEL SISTEMA

El sistema implementa un procesador digital de señales completo en tiempo real sobre la placa **Arduino UNO Q**, la cual cuenta con una arquitectura heterogénea de doble núcleo:
1. **Microcontrolador STM32U585 (160 MHz)** ejecutando **Zephyr RTOS**: Se encarga de las tareas determinísticas en tiempo real estricto (muestreo ADC de 12 bits a 1000 Hz, ejecución de filtros digitales IIR y generación analógica mediante DAC de 12 bits).
2. **Microprocesador Linux Cortex-A53** ejecutando **Python 3**: Se encarga del procesamiento matemático intensivo (Transformada Rápida de Fourier RFFT, detección de armónicas, cálculo de THD, servidor WebSocket y puente IPC).
3. **Interfaz Web (Navegador)**: Funciona como centro de control y osciloscopio digital interactivo con escalas de tiempo, gráficos en vivo y selectores de parámetros.

### Diagrama en Bloques del Flujo de Datos

```
[ Generador de Señales ]
  (12 Vpp: -6V a +6V, 1 Hz a 100 Hz)
            │
            ▼
[ Circuito Acondicionador ]
  (Divisor R1=12k, R2=10k, R3=8.2k + Buffer OpAmp LM324)
  Escalado a rango seguro: 0.00 V a 3.28 V
            │
            ▼
[ Microcontrolador STM32U585 - Zephyr RTOS ]
  ├── Hilo RTOS Prioritario (Prioridad 1, periodo 1 ms = 1000 Hz):
  │     ├─ Lectura ADC 12 bits (Pin A4)
  │     ├─ Conversión a tensión matemática: Vin = (Vadc - 1.638) * 3.664
  │     ├─ Filtro Digital IIR (Bypass, Pasabajas, Pasaaltos o Pasabanda)
  │     ├─ Conversión a escala DAC: Vout -> DAC (0 a 4095)
  │     ├─ Escritura analógica en DAC0 (hacia Osciloscopio externo)
  │     └─ Inserción en Buffer Circular Lock-Free (1024 posiciones)
  │
  └── Hilo Bridge RPC (Prioridad 5):
        └─ Comunicación asíncrona hacia Linux por socket IPC
            │
            ▼
[ Microprocesador Linux Cortex-A53 - Python ]
  ├── Lectura de lotes de muestras desde el buffer del microcontrolador
  ├── Acumulación en buffer FIFO de N = 1024 muestras (1.024 segundos)
  ├── Preprocesamiento: Remoción de valor medio DC + Ventana de Hanning
  ├── Cálculo de RFFT con compensación de ganancia coherente
  ├── Interpolación parabólica de picos (resolución sub-binaria)
  ├── Detección de Fundamental, 2ª armónica, 3ª armónica y THD
  └── Emisión de telemetría a la interfaz web (~25 cuadros por segundo)
            │
            ▼
[ Interfaz Web de Usuario ]
  ├── Osciloscopio con canal CH1 (Entrada) y canal CH2 (Salida DAC)
  ├── HUD con mediciones automáticas (Vpp, Vrms, Frecuencia, Periodo)
  ├── Base de tiempo configurable (20 ms a 1000 ms) y selector de escala
  ├── Tabla dinámica de armónicas y gráfico de espectro FFT
  └── Control de filtros digitales (tipo y frecuencia de corte fc)
```

---

## 2. DESGLOSE CASO POR CASO: TEORÍA Y CÓMO LO RESOLVEMOS

---

### CASO 1: Acondicionamiento de Señal de Entrada (-6V a +6V) y Ganancia del Sistema

#### A) La Teoría
La señal entregada por el generador de laboratorio tiene una amplitud de 12 Vpp (oscila de -6.0 V a +6.0 V). Los pines analógicos del microcontrolador STM32 solo admiten tensiones positivas dentro del rango de 0.0 V a 3.3 V. Conectar la señal directamente quemaría el circuito integrado por sobretensión o polarización inversa.

Por lo tanto, se requiere un circuito de adaptación lineal con dos funciones:
1. **Atenuación**: Reducir los 12 V de excursión a un rango menor o igual a 3.3 V.
2. **Desplazamiento de Nivel (Offset DC)**: Centrar la señal alrededor de la mitad de la escala (aproximadamente 1.65 V) para que el mínimo (-6 V) coincida con 0 V y el máximo (+6 V) coincida con el tope positivo.

#### B) Cómo lo Resolvemos Nosotros
Diseñamos un triángulo de resistencias convergentes en un nodo central (Vadc):
* **R1 = 12 kΩ** conectada a la entrada del generador (Vin).
* **R2 = 10 kΩ** conectada a la fuente de referencia de +5.0 V (suministra el offset).
* **R3 = 8.2 kΩ** conectada a masa GND (drena corriente para fijar la ganancia).
* La salida del nodo se conecta a un amplificador operacional LM324 en configuración de seguidor de tensión (buffer) para aislar la impedancia y proteger el conversor ADC.

**Deducción matemática en el nodo:**
Aplicando la Ley de Corrientes de Kirchhoff (la suma de corrientes que entran al nodo es igual a cero):

```
(Vin - Vadc)/R1 + (5V - Vadc)/R2 + (0V - Vadc)/R3 = 0

Vadc = [ (Vin / R1) + (5V / R2) ] / [ (1 / R1) + (1 / R2) + (1 / R3) ]
```

Calculando la conductancia total del nodo:
* G_total = (1 / 12000) + (1 / 10000) + (1 / 8200) = 0.00008333 + 0.00010000 + 0.00012195 = 0.00030528 Siemens

Evaluando para los puntos clave del generador:
1. **Para Vin = -6.0 V (Pico mínimo):**
   * Corriente desde R1: -6.0 / 12000 = -0.0005 A
   * Corriente desde R2 (+5V): +5.0 / 10000 = +0.0005 A
   * Suma de corrientes: -0.0005 + 0.0005 = 0 A.
   * **Vadc = 0.0000 V** (cero voltios exactos).
2. **Para Vin = 0.0 V (Cruce por cero):**
   * Vadc = (0 + 0.0005) / 0.00030528 = **1.6378 V** (punto medio exacto).
3. **Para Vin = +6.0 V (Pico máximo):**
   * Vadc = ( (6 / 12000) + 0.0005 ) / 0.00030528 = 0.0010 / 0.00030528 = **3.2756 V**.

El rango queda comprendido entre **0.00 V y 3.28 V**, aprovechando el 99.3% del rango dinámico del conversor ADC de 3.3 V sin riesgo de saturación.

**Ecuaciones de conversión en el código C del microcontrolador:**
* Para recuperar la tensión física original:
  ```
  V_ard = (ADC_leido / 4095.0) * 3.3 V
  Vin = (V_ard - 1.6378 V) * 3.6635
  ```
* Para enviar la señal resultante procesada al DAC físico (0 a 3.3 V):
  ```
  V_dac = (Vout / 3.6635) + 1.6378 V
  valor_dac = (int)((V_dac / 3.3 V) * 4095.0)
  ```

**Ganancia del sistema y variación por 1V:**
* Amplitud de entrada: 12.0 Vpp (-6V a +6V).
* Amplitud de salida física del DAC: 3.3 Vpp (0V a 3.3V).
* **Ganancia Global (A)**:
  `A = Vout_pp / Vin_pp = 3.3 V / 12.0 V = 0.275 V/V  (-11.21 dB)`
* **Variación en salida por cada 1V en entrada**:
  `ΔVout = 1.0 V * 0.275 = 0.275 V = 275 mV`

---

### CASO 2: El "Bache" en el Osciloscopio y la Pérdida de Muestras

#### A) La Teoría y Causa del Problema Original
En sistemas embebidos de procesamiento en tiempo real (DSP), la adquisición de muestras y la actualización del conversor digital a analógico (DAC) deben realizarse a intervalos de tiempo estrictamente constantes (período de muestreo Ts = 1/Fs exacto). Si el procesador suspende la tarea de muestreo para atender otras operaciones (como la comunicación serial o la red), el DAC mantiene congelada su tensión anterior. En el osciloscopio, esto se manifiesta como un escalón horizontal plano (bache) que distorsiona la morfología continua de la onda.

En el código original ocurrían dos fallas críticas:
1. **Sobrecarga del buffer RPC**: La librería de comunicación `Arduino_RPClite` tiene un límite rígido de paquete de 1024 bytes (`DECODER_BUFFER_SIZE = 1024`). Cuando se intentaba acumular y transmitir lotes de muestras en punto flotante, el buffer se desbordaba y el microcontrolador descartaba datos.
2. **Lazo bloqueado en loop()**: El muestreo se controlaba por software con `micros()` dentro del `loop()` general. Cuando llegaba una solicitud RPC desde Python, el hilo de comunicación de Zephyr (prioridad 5) acaparaba la CPU para serializar paquetes MsgPack. Si esa rutina demoraba más de 5 ms, la condición de guarda en el código reseteaba el tiempo (`t_ultimo_muestreo = t_actual`), descartando de golpe 5 a 10 muestras continuas. Durante ese lapso, el DAC no se actualizaba, generando el bache periódico.

#### B) Cómo lo Resolvemos Nosotros
1. **Hilo prioritario en tiempo real sobre Zephyr RTOS**:
   Separamos completamente la adquisición del `loop()`. Creamos un hilo del sistema operativo en tiempo real (`k_thread_create`) con **Prioridad 1**, la cual es estrictamente superior a la prioridad del módulo de comunicación Bridge (Prioridad 5).
   Este hilo se despierta de manera determinística cada 1.0 milisegundo mediante el temporizador del núcleo del sistema:
   ```cpp
   next_tick += 1;
   k_sleep(K_TIMEOUT_ABS_MS(next_tick));
   ```
   Cuando llega el tick de 1 ms, el hilo prioritario interrumpe cualquier otra tarea, lee el ADC, ejecuta el filtro IIR, escribe el nuevo valor en el DAC en menos de 20 microsegundos y vuelve a dormir. **El DAC nunca se interrumpe, garantizando una salida analógica continua, suave y sin baches**.

2. **Buffer Circular Lock-Free (1024 posiciones)**:
   Para independizar la velocidad de muestreo del microcontrolador (1000 Hz) de la frecuencia de lectura de Python (~25 Hz), implementamos un buffer circular de 1024 muestras en memoria RAM.
   * El hilo de muestreo escribe datos como enteros con signo de 16 bits (`int16_t`) expresados en milivoltios.
   * Python lee lotes controlados de 64 muestras (128 bytes de datos, perfectamente dentro del límite seguro de 1024 bytes de RPClite).
   * Al ser un buffer circular de productor único y consumidor único, no requiere mutexes bloqueantes, asegurando cero pérdida de datos.

---

### CASO 3: La Distorsión a partir de 30 Hz y la Frecuencia de Muestreo a 1000 Hz

#### A) La Teoría
El **Teorema de Muestreo de Nyquist-Shannon** establece que para poder reconstruir unívocamente una señal analógica a partir de sus muestras discretas, la frecuencia de muestreo Fs debe ser mayor que el doble de la máxima frecuencia contenida en la señal (Fmax):
`Fs > 2 * Fmax`

Sin embargo, hay una diferencia fundamental entre el límite teórico de reconstrucción y la representación visual fluida:
* Si se muestrea con solo 2 puntos por período (límite exacto de Nyquist), la onda dibujada con líneas rectas parece un triángulo o una línea quebrada desfigurada.
* Para representar una onda senoidal en un osciloscopio digital sin distorsión visual perceptible por el ojo humano se requieren al menos **10 muestras por ciclo**.

En el código previo, la telemetría enviada a la web estaba diezmada a `Fs = 200 Hz`:
* A 10 Hz: 20 muestras por ciclo (se veía bien).
* A 30 Hz: 200 / 30 = 6.6 muestras por ciclo (comenzaba a verse facetada y con batido).
* A 50 Hz: 4 muestras por ciclo (parecía un rombo o triángulo distorsionado).
* A 100 Hz: 2 muestras por ciclo (límite crítico, la amplitud oscilaba artificialmente dependiendo del ángulo de fase del muestreo).

Además, en el script de Python la búsqueda de la fundamental estaba restringida artificialmente por código a un máximo de 35 Hz:
`idx_max = min(len(magnitudes) - 2, int(35.0 / df))`
Cualquier señal inyectada por encima de 30 Hz no podía ser localizada por el algoritmo, cayendo en lecturas de ruido o frecuencias erróneas.

#### B) Cómo lo Resolvemos Nosotros
1. **Muestreo unificado a 1000 Hz nativos**:
   Elevamos la tasa efectiva de adquisición a **Fs = 1000 Hz**, fijando la frecuencia de Nyquist en **500 Hz**.
   * Una señal de 10 Hz tiene 100 muestras por período.
   * Una señal de 50 Hz tiene 20 muestras por período.
   * Una señal de 100 Hz tiene 10 muestras completas por período. Con 10 muestras por ciclo y la interpolación cúbica del osciloscopio web, la señal de 100 Hz se visualiza con perfecta nitidez senoidal.
2. **Ampliación del rango de búsqueda en Python**:
   Eliminamos el tope de 35 Hz y configuramos el algoritmo para buscar la fundamental entre 1.0 Hz y 125.0 Hz, cubriendo holgadamente todo el rango del trabajo práctico.

---

### CASO 4: Cálculo de la Transformada Rápida de Fourier (FFT), Ventaneo y Armónicas

#### A) La Teoría
La Transformada Discreta de Fourier (DFT) transforma una secuencia de N muestras en el dominio del tiempo en sus componentes espectrales en el dominio de la frecuencia.

Para implementarla con rigor profesional en un sistema de medición se deben resolver tres fenómenos físicos:
1. **Fuga Espectral (Spectral Leakage)**: La FFT asume que el bloque de N muestras se repite infinitamente en el tiempo. Si la señal no contiene un número entero exacto de períodos dentro del bloque, el inicio y el final de la ventana no encajan, creando un salto discontinuo. Este salto artificial introduce armónicas inexistentes que ensucian el espectro.
2. **Ventaneo de Hanning**: Se multiplica la señal temporal por una función suave de ventana que vale cero en los bordes y uno en el centro:
   `w[n] = 0.5 - 0.5 * cos(2 * π * n / N)`
   Esto anula las discontinuidades de los bordes y suprime drásticamente la fuga espectral hacia los bins laterales.
3. **Pérdida de Ganancia Coherente y Pérdida de Festoneado (Scalloping Loss)**:
   * Al atenuar los bordes, la ventana reduce la energía total de la señal a la mitad. Para leer la amplitud real pico (Vp) se debe dividir por la ganancia coherente de la ventana:
     `G_coh = sumatoria(w[n]) / N ≈ 0.5`
   * Si la frecuencia de la señal cae entre dos canales discretos de la FFT (entre el bin k y el bin k+1), la amplitud medida en el bin resulta menor que la real.

#### B) Cómo lo Resolvemos Nosotros
1. **Adquisición de bloque N = 1024 muestras**:
   A Fs = 1000 Hz, 1024 muestras corresponden a T = 1.024 segundos de señal.
   La separación entre bins espectrales es:
   `Δf = Fs / N = 1000 Hz / 1024 ≈ 0.9766 Hz por bin`
2. **Remoción de la componente continua (DC)**:
   Se calcula la media de las 1024 muestras y se resta a cada valor:
   `x_ac[n] = x[n] - media(x)`
   Esto asegura que el bin 0 (0 Hz) quede en 0 V y no enmascare frecuencias bajas.
3. **Ventana de Hanning y Transformada Real (RFFT)**:
   Se aplica la ventana de Hanning y se calcula la FFT para señales reales (`numpy.fft.rfft`), la cual devuelve N/2 + 1 coeficientes (desde 0 Hz hasta 500 Hz).
   Se escala dividiendo por la suma de la ventana multiplicada por 2:
   `magnitudes = abs(RFFT(x * w)) * (2.0 / sumatoria(w))`
4. **Interpolación Parabólica Cuadrática de Picos**:
   Para obtener precisión decimal sin requerir un bloque gigantesco de muestras, identificamos el bin k de mayor magnitud y sus dos vecinos contiguos (y1 = bin k-1, y2 = bin k, y3 = bin k+1). Ajustamos una parábola centrada:
   ```
   denominador = y1 - 2*y2 + y3
   delta = 0.5 * (y1 - y3) / denominador
   frecuencia_exacta = (k + delta) * Δf
   ```
   Luego se corrige la amplitud dividiendo por el factor sinc de Hanning:
   `Amplitud_Vp = y2 / [ sinc(delta) / (1 - delta^2) ]`
5. **Detección de 2ª y 3ª Armónica**:
   Con la fundamental f1 calculada, el algoritmo proyecta 2*f1 y 3*f1. Busca en una ventana de +/- 3 bins alrededor de esa posición teórica y confirma si existe un pico real sobre el piso de ruido.
6. **Distorsión Armónica Total (THD)**:
   Se calcula automáticamente con las amplitudes pico obtenidas:
   `THD = [ sqrt( (A2)^2 + (A3)^2 ) / A1 ] * 100%`

---

### CASO 5: Filtros Digitales IIR en Tiempo Real y Generación DAC

#### A) La Teoría
Para procesar la señal muestra a muestra con latencia casi nula en el microcontrolador, se emplean filtros de **Respuesta Infinita al Impulso (IIR)** de primer orden. Estos filtros se derivan a partir de la ecuación diferencial de un circuito analógico RC mediante la aproximación en diferencias discretas con paso Ts = 0.001 s (1 ms).

* Constante de tiempo RC en función de la frecuencia de corte elegida fc:
  `RC = 1 / (2 * π * fc)`

1. **Filtro Pasabajas (LPF)**:
   Deja pasar frecuencias menores a fc y atenúa las superiores a razón de -20 dB por década.
   * Factor de suavizado: `α = Ts / (RC + Ts)`
   * Ecuación en diferencias:
     `y[n] = α * x[n] + (1 - α) * y[n-1]`
2. **Filtro Pasaaltos (HPF)**:
   Bloquea el valor continuo y frecuencias inferiores a fc, dejando pasar frecuencias altas.
   * Factor de suavizado: `α = RC / (RC + Ts)`
   * Ecuación en diferencias:
     `y[n] = α * ( y[n-1] + x[n] - x[n-1] )`
3. **Filtro Pasabanda (BPF)**:
   Formado por una cascada donde la salida de la primera etapa pasabajas ingresa a la segunda etapa pasaaltos:
   * Ancho de banda: `BW = max(4 Hz, 0.3 * fc)`
   * Etapa 1 (Pasabajas en fc + BW/2): `w[n] = α_lp * x[n] + (1 - α_lp) * w[n-1]`
   * Etapa 2 (Pasaaltos en fc - BW/2): `y[n] = α_hp * ( y[n-1] + w[n] - w[n-1] )`

#### B) Cómo lo Resolvemos Nosotros en el Microcontrolador
* Los coeficientes se precalculan únicamente cuando el usuario cambia el filtro o la frecuencia desde la web mediante la llamada RPC `configurarFiltro(tipo, fc)`.
* Durante el ciclo de 1 ms del hilo prioritario, la función de filtrado solo ejecuta 2 multiplicaciones y 1 suma en coma flotante nativa de 32 bits, insumiendo menos de 2 microsegundos de CPU.
* El resultado filtrado se convierte a código DAC (0 a 4095) con saturación estricta para evitar desbordes y se escribe al pin analógico físico `DAC0` mediante `analogWrite(DAC0, valor)`.

---

## 3. GUÍA EXHAUSTIVA DE ENSAYOS: QUÉ SE OBSERVA EN LA GRÁFICA Y EL OSCILOSCOPIO

A continuación se detalla exactamente qué ocurre y qué se visualiza en pantalla para cada señal inyectada (12 Vpp: -6V a +6V), variando su frecuencia y la frecuencia de corte del filtro.

---

### 3.1. ENSAYO CON SEÑAL SENOIDAL (12 Vpp / 6 Vp)

Una señal senoidal pura ideal posee toda su energía concentrada en una única frecuencia:
* Ecuación temporal: `x(t) = 6.0 * sin(2 * π * f0 * t)`
* Espectro FFT: Un único pico en f0 con amplitud 6.0 Vp (12.0 Vpp).
* Armónicas teóricas: 2ª armónica = 0.0 V, 3ª armónica = 0.0 V, THD ≈ 0.0%.

#### Tabla de Comportamiento con Filtro Pasabajas (LPF)
| Frecuencia Señal (f0) | Frecuencia de Corte (fc) | Relación | Qué se observa en el Osciloscopio y la Gráfica Web | Mediciones en Pantalla |
| :--- | :--- | :--- | :--- | :--- |
| **10 Hz** (Baja) | **80 Hz** | fc >> f0 | La senoide atraviesa el filtro intacta. Las curvas CH1 (Entrada) y CH2 (Salida) están superpuestas en fase y amplitud. | Vin = 12.0 Vpp, Vout = 12.0 Vpp, Ganancia = 1.00, f = 10.0 Hz. |
| **30 Hz** (Media) | **30 Hz** | fc = f0 | **Punto de corte a -3 dB**: La amplitud de salida se reduce al 70.7% del valor de entrada (cae de 6.0 Vp a 4.24 Vp). La onda de salida presenta un retardo temporal de fase de -45 grados. | Vin = 12.0 Vpp, Vout = 8.5 Vpp, Ganancia = 0.71, desfase visible. |
| **50 Hz** (Media) | **10 Hz** | fc << f0 | La señal se encuentra muy por encima de la frecuencia de corte. La onda senoidal de salida se atenúa severamente, viéndose casi como una línea horizontal cercana a 0 V. | Vin = 12.0 Vpp, Vout ≈ 2.3 Vpp, Ganancia ≈ 0.19. |
| **100 Hz** (Alta) | **100 Hz** | fc = f0 | Con la base de tiempo en 20 ms se observan dos ciclos senoidales nítidos (10 muestras por ciclo). La amplitud cae al 70.7% (8.5 Vpp) y la fase se retrasa 45 grados. | Vin = 12.0 Vpp, Vout = 8.5 Vpp, Ganancia = 0.71, f = 100.0 Hz. |

#### Tabla de Comportamiento con Filtro Pasaaltos (HPF)
| Frecuencia Señal (f0) | Frecuencia de Corte (fc) | Relación | Qué se observa en el Osciloscopio y la Gráfica Web | Mediciones en Pantalla |
| :--- | :--- | :--- | :--- | :--- |
| **10 Hz** (Baja) | **60 Hz** | fc >> f0 | La frecuencia de la señal es bloqueada por estar en la zona de rechazo de bajas frecuencias. La salida es una línea plana en 0 V. | Vin = 12.0 Vpp, Vout ≈ 1.9 Vpp, Ganancia ≈ 0.16. |
| **50 Hz** (Media) | **50 Hz** | fc = f0 | **Punto de corte a -3 dB**: Amplitud al 70.7% (8.5 Vpp). La salida presenta un adelanto de fase de +45 grados respecto a la entrada. | Vin = 12.0 Vpp, Vout = 8.5 Vpp, Ganancia = 0.71. |
| **100 Hz** (Alta) | **20 Hz** | fc << f0 | La señal de 100 Hz está plenamente dentro de la banda de paso. La onda atraviesa el filtro sin reducción de amplitud ni desfase. | Vin = 12.0 Vpp, Vout = 12.0 Vpp, Ganancia = 1.00. |

#### Tabla de Comportamiento con Filtro Pasabanda (BPF)
| Frecuencia Señal (f0) | Frecuencia de Corte (fc) | Relación | Qué se observa en el Osciloscopio y la Gráfica Web | Mediciones en Pantalla |
| :--- | :--- | :--- | :--- | :--- |
| **40 Hz** | **40 Hz** | fc = f0 | Máxima transmisión. La señal de salida coincide en frecuencia y conserva prácticamente toda la amplitud de entrada. | Vin = 12.0 Vpp, Vout ≈ 11.5 Vpp, f = 40.0 Hz. |
| **40 Hz** | **15 Hz** o **80 Hz** | fc desintonizada | La señal queda fuera de la ventana de paso. La onda senoidal de salida se atenúa enérgicamente hacia 0 V. | Vout < 2.0 Vpp, Ganancia < 0.15. |

---

### 3.2. ENSAYO CON SEÑAL CUADRADA (12 Vpp / 6 Vp)

La onda cuadrada simétrica (duty cycle 50%) se descompone según su Serie de Fourier en la suma de armónicas impares:
```
x(t) = (4 * A / π) * [ sin(ω0*t) + (1/3)*sin(3*ω0*t) + (1/5)*sin(5*ω0*t) + (1/7)*sin(7*ω0*t) + ... ]
```
Para A = 6.0 V (amplitud de -6V a +6V):
* **Fundamental (f0)**: Amplitud pico = (4 * 6) / π ≈ **7.64 Vp (15.28 Vpp)**. (La fundamental supera el valor pico de la onda cuadrada debido al efecto de cancelación armónica en los valles).
* **2ª Armónica (2*f0)**: Nula (0.0 V) por simetría de media onda.
* **3ª Armónica (3*f0)**: Amplitud pico = 7.64 / 3 ≈ **2.55 Vp (5.09 Vpp)**, correspondiente al 33.3% de la fundamental.
* **5ª Armónica (5*f0)**: Amplitud pico = 7.64 / 5 ≈ 1.53 Vp (20.0% de la fundamental).
* **THD Teórico**: Mayor al 33.3%.

#### Qué se Observa en Gráfica y Osciloscopio según el Filtro:

#### 1. Sin Filtro (Bypass directo):
* **Osciloscopio**: Onda cuadrada con transiciones verticales rápidas y niveles superior e inferior planos.
* **Tabla FFT**: Muestra Fundamental f0 con ~7.6 Vp, 2ª Armónica en 0 V, y 3ª Armónica (en 3*f0) con ~2.5 Vp (33% de la fundamental). THD marcado en ~33%.

#### 2. Filtro Pasabajas (LPF):
* **Caso fc >> 3*f0 (ej: f0 = 10 Hz, fc = 80 Hz)**:
  Pasan la fundamental (10 Hz), la 3ª armónica (30 Hz) y la 5ª armónica (50 Hz). En el osciloscopio la señal conserva la forma cuadrada pero sus esquinas superiores e inferiores presentan un leve redondeo y una pequeña ondulación (fenómeno de Gibbs amortiguado).
* **Caso f0 < fc < 3*f0 (EL ENSAYO CLAVE DE LA DEFENSA: ej. f0 = 20 Hz, fc = 25 Hz)**:
  La frecuencia de corte permite el paso libre de la fundamental de 20 Hz, pero bloquea enérgicamente la 3ª armónica de 60 Hz y todas las superiores.
  * **Efecto visual asombroso**: La onda cuadrada pierde completamente sus bordes rectos y **se transforma físicamente en una onda senoidal pura y suave de 20 Hz**.
  * **En la tabla FFT**: La 3ª armónica cae de 2.5 Vp a prácticamente 0.0 V, y el THD medido se desploma desde el 33% hasta menos del 3%. Es la comprobación empírica directa del Teorema de Fourier.
* **Caso fc << f0 (ej: f0 = 50 Hz, fc = 10 Hz)**:
  Tanto la fundamental como todas las armónicas son atenuadas. La señal se reduce a una línea recta casi imperceptible en 0 V.

#### 3. Filtro Pasaaltos (HPF):
* **Caso fc << f0 (ej: f0 = 50 Hz, fc = 5 Hz)**:
  Pasan la fundamental y todas las armónicas altas. La onda cuadrada atraviesa el sistema conservando su forma.
* **Caso fc ≈ f0 (ej: f0 = 10 Hz, fc = 10 Hz)**:
  Las mesetas horizontales de la onda cuadrada ya no se ven planas: la tensión decae exponencialmente durante el semiperíodo positivo y sube durante el negativo (efecto de "droop" o techo inclinado característico de la diferenciación parcial).
* **Caso fc >> f0 (OTRO ENSAYO DE ALTO IMPACTO: ej. f0 = 5 Hz, fc = 40 Hz)**:
  El filtro elimina la fundamental lenta y los tramos planos de continua (0 Hz local). Solo permite el paso de las frecuencias ultra rápidas que componen las transiciones abruptas de subida y bajada.
  * **Efecto visual**: La onda cuadrada se convierte en una serie de **espigas / agujas exponenciales bidireccionales**: un pulso agudo positivo en cada flanco ascendente y un pulso agudo negativo en cada flanco descendente. **Es la derivada matemática de la onda cuadrada**.

#### 4. Filtro Pasabanda (BPF):
* **Sintonizado en la fundamental (fc = f0 = 25 Hz)**:
  El filtro suprime las armónicas impares superiores y extrae la componente fundamental: la onda cuadrada se convierte en una senoide pura a 25 Hz.
* **Sintonizado en la 3ª Armónica (fc = 3*f0: ej. f0 = 20 Hz, sintonizamos fc = 60 Hz)**:
  El filtro rechaza la fundamental de 20 Hz por baja frecuencia y elimina frecuencias superiores a 70 Hz.
  * **Efecto visual**: En el osciloscopio y en la pantalla se observa **una onda senoidal perfecta de 60 Hz (el triple de frecuencia que la señal que entrega el generador)** con una amplitud de 2.5 Vp. Esto demuestra visualmente que las armónicas existen físicamente dentro de la onda cuadrada.

---

### 3.3. ENSAYO CON SEÑAL TRIANGULAR (12 Vpp / 6 Vp)

La onda triangular simétrica posee únicamente armónicas impares, pero su energía decae con el inverso del cuadrado del armónico (1 / k^2):
```
x(t) = (8 * A / π^2) * [ sin(ω0*t) - (1/9)*sin(3*ω0*t) + (1/25)*sin(5*ω0*t) - ... ]
```
Para A = 6.0 V (amplitud de -6V a +6V):
* **Fundamental (f0)**: Amplitud pico = (48 / π^2) ≈ **4.86 Vp (9.73 Vpp)**.
* **2ª Armónica (2*f0)**: Nula (0.0 V).
* **3ª Armónica (3*f0)**: Amplitud pico = 4.86 / 9 ≈ **0.54 Vp (1.08 Vpp)**, representando solo el **11.1%** de la fundamental.
* **5ª Armónica (5*f0)**: Amplitud pico = 4.86 / 25 ≈ 0.19 Vp (3.9% de la fundamental).
* **THD Teórico**: Aproximadamente **12.1%**.

#### Qué se Observa en Gráfica y Osciloscopio según el Filtro:

#### 1. Sin Filtro (Bypass directo):
* **Osciloscopio**: Señal triangular con pendientes rectas y vértices puntiagudos en +6V y -6V.
* **Tabla FFT**: Fundamental en f0 con ~4.9 Vp, 2ª Armónica en 0 V, 3ª Armónica en ~0.5 Vp (11% de la fundamental), THD reportado en ~12%.

#### 2. Filtro Pasabajas (LPF):
* **Caso fc >> f0 (ej: f0 = 15 Hz, fc = 80 Hz)**:
  Pasan la fundamental y la 3ª armónica. La onda conserva su forma triangular con vértices ligeramente suavizados.
* **Caso f0 < fc < 3*f0 (ej: f0 = 20 Hz, fc = 30 Hz)**:
  Como la 3ª armónica solo aportaba el 11% de la forma original, al filtrarla la señal se convierte casi de inmediato en una onda senoidal pura de amplitud 4.86 Vp.
* **Caso fc << f0**:
  Atenuación total hacia 0 V.

#### 3. Filtro Pasaaltos (HPF):
* **Caso fc << f0**:
  La onda triangular atraviesa el filtro sin modificaciones.
* **Caso fc >> f0 (ej: f0 = 10 Hz, fc = 60 Hz)**:
  El filtro pasaaltos actúa como un diferenciador. La derivada temporal de una rampa lineal ascendente es un valor constante positivo, y la de una rampa descendente es un valor constante negativo.
  * **Efecto visual**: Al eliminar las frecuencias bajas y derivar la señal, **la onda triangular se transforma en una onda cuadrada / rectangular desfasada 90 grados**.

#### 4. Filtro Pasabanda (BPF):
* **Sintonizado en fc = f0**:
  Aísla la fundamental, mostrando una senoide limpia a f0.
* **Sintonizado en fc = 3*f0**:
  Aísla la 3ª armónica: se visualiza una pequeña onda senoidal a tres veces la frecuencia del generador, con amplitud de ~0.54 Vp.

---

## 4. CUADRO RESUMEN DE COMPORTAMIENTO GENERAL

| Filtro | Relación de Frecuencias | Efecto en Senoidal | Efecto en Cuadrada | Efecto en Triangular |
| :--- | :--- | :--- | :--- | :--- |
| **Pasabajas (LPF)** | **fc >> f0** | Pasa intacta (Ganancia 1.0) | Pasa cuadrada con bordes redondeados | Pasa triangular casi idéntica |
| **Pasabajas (LPF)** | **f0 < fc < 3*f0** | Pasa senoide con leve retraso | **Se transforma en Senoide pura** (filtra armónicas) | **Se transforma en Senoide pura** |
| **Pasabajas (LPF)** | **fc << f0** | Se atenúa a línea plana (0V) | Se atenúa a línea plana (0V) | Se atenúa a línea plana (0V) |
| **Pasaaltos (HPF)** | **fc << f0** | Pasa intacta (Ganancia 1.0) | Pasa cuadrada intacta | Pasa triangular intacta |
| **Pasaaltos (HPF)** | **fc ≈ f0** | Cae a 70.7% (-3dB), adelanto +45° | Techos inclinados (droop exponencial) | Deformación en pendientes |
| **Pasaaltos (HPF)** | **fc >> f0** | Se atenúa a 0V | **Se convierte en espigas / pulsos (derivada)** | **Se convierte en onda cuadrada (derivada)** |
| **Pasabanda (BPF)** | **fc = f0** | Pasa senoide pura | Pasa fundamental senoidal | Pasa fundamental senoidal |
| **Pasabanda (BPF)** | **fc = 3*f0** | Se atenúa a 0V | **Aísla 3ª armónica (Senoide a 3*f0)** | **Aísla 3ª armónica (Senoide a 3*f0)** |

---

## 5. GUÍA RÁPIDA DE PREGUNTAS Y RESPUESTAS PARA EL COLOQUIO

### P1: ¿Por qué en las pruebas iniciales el osciloscopio mostraba un escalón o bache periódico en la señal de salida del DAC?
> **Respuesta**: "Porque el lazo de muestreo se ejecutaba en la función general `loop()`. Cuando Python solicitaba telemetría por llamadas RPC, el módulo de comunicación de Zephyr consumía tiempo de procesador para armar y transmitir los paquetes. Si esa operación tardaba más de 5 ms, el código descartaba las muestras que debieron tomarse en ese lapso y el DAC mantenía congelada la tensión anterior. Lo resolvimos creando un **hilo nativo en Zephyr RTOS con Prioridad 1** (más alta que el Bridge de comunicación, que tiene prioridad 5). Cada 1 ms exacto, el hilo prioritario toma el control por 20 microsegundos, procesa la muestra, actualiza el DAC y la almacena en un buffer circular de 1024 posiciones. Así, el DAC jamás se interrumpe y la señal es continua y perfecta."

### P2: ¿Por qué antes a partir de 30 Hz la señal se distorsionaba en la página web?
> **Respuesta**: "Había dos causas: primero, la telemetría estaba diezmada a 200 Hz, lo que daba apenas 6 muestras por ciclo a 30 Hz y solo 2 muestras a 100 Hz, provocando una deformación poligonal de la onda. Segundo, en Python la búsqueda de la fundamental tenía un límite fijo en el código de 35 Hz (`idx_max = int(35.0 / df)`), por lo que cualquier frecuencia superior era ignorada. Al llevar la frecuencia de muestreo a 1000 Hz, el límite de Nyquist subió a 500 Hz: una señal de 100 Hz cuenta con 10 muestras por período y sus armónicas de 200 Hz y 300 Hz se miden con absoluta claridad sin aliasing."

### P3: ¿Por qué es indispensable aplicar la ventana de Hanning antes de la FFT?
> **Respuesta**: "Porque la FFT asume que el bloque de 1024 muestras se repite cíclicamente hasta el infinito. Como la frecuencia de la señal analógica real no es un submúltiplo exacto de la ventana temporal, los extremos del bloque no coinciden, generando una discontinuidad brusca de tensión. La transformada interpreta ese escalón como frecuencias altas inexistentes que ensucian todo el espectro (fuga espectral). La ventana de Hanning multiplica la señal por una curva que desciende suavemente a cero en los extremos, eliminando la discontinuidad y confinando la energía al pico real."

### P4: ¿Por qué la fundamental de una onda cuadrada de 12 Vpp mide aproximadamente 15.3 Vpp en la FFT?
> **Respuesta**: "Porque de acuerdo con el desarrollo en Serie de Fourier, el primer armónico de una onda cuadrada simétrica tiene una amplitud pico de `(4 / π) * A`, es decir, 1.273 veces la amplitud pico de la onda cuadrada. Para 6 Vp (12 Vpp), la fundamental pura mide `1.273 * 6 V = 7.64 Vp` (15.28 Vpp). Esto no es un error de medición; es la demostración física de que las armónicas impares superiores restan amplitud en el centro del semiperíodo para aplanar la meseta de la onda cuadrada."
