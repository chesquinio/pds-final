# Sistema de Procesamiento Digital de Señales (DSP) - Arduino UNO Q

Proyecto desarrollado para la cátedra de **Procesamiento Digital de Señales (PDS)**, Universidad Nacional de Rafaela (UNRaf).

Implementa un procesador digital de señales completo en tiempo real combinando:
1. **Microcontrolador STM32U585 (Zephyr RTOS)**: Adquisición ADC (12 bits) @ 1000 Hz, filtros digitales IIR en tiempo real (Pasabajas, Pasaaltos, Pasabanda) y reconstrucción analógica continua mediante DAC (12 bits) hacia osciloscopio externo.
2. **Microprocesador Linux Cortex-A53 (Python)**: Análisis espectral FFT con ventana de Hanning, interpolación parabólica de armónicas (fundamental, 2ª y 3ª armónica), cálculo de THD, servidor WebSocket y puente IPC.
3. **Interfaz Gráfica Web**: Osciloscopio digital en tiempo real con selector de base de tiempo (20 ms a 1000 ms), medición automática de parámetros (Vpp, Vrms, Frecuencia, T0), congelamiento de pantalla y control dinámico de filtros digitales.

---

## Documentación Completa para la Defensa Técnica

Para consultar la fundamentación teórica matemática, el desarrollo de las ecuaciones de Fourier, el comportamiento analítico de las señales (senoidal, cuadrada y triangular) y la guía de respuestas para el coloquio oral, ver:
* [DOCUMENTACION_DSP.md](DOCUMENTACION_DSP.md)

---

## Estructura del Repositorio

* `sketch/`: Código en C++ para el microcontrolador STM32 (Zephyr RTOS).
  * `sketch.ino`: Hilo RTOS determinístico a 1000 Hz, filtros IIR, escritura DAC y buffer circular lock-free.
* `python/`: Backend en Python para el procesador Linux.
  * `main.py`: Gestión de IPC/RPC, cálculo de RFFT, detección de armónicas y servidor WebUI.
  * `requirements.txt`: Dependencias de Python.
* `assets/`: Frontend web interactivo.
  * `index.html`: Interfaz del osciloscopio, tabla dinámica de armónicas y selector de filtros.
* `app.yaml`: Configuración de despliegue de Arduino App Lab.
