#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <vector>
#include <stdint.h>
#include <math.h>

#define PIN_DAC DAC0
const int pinADC = A4;

// Conversión física de niveles de tensión (+/- 6V <-> 0..3.3V)
// R1 = 12k (entrada), R2 = 10k (+5V), R3 = 8.2k (GND)
// Para Vin = -6V -> V_adc = 0.00V
// Para Vin = 0V  -> V_adc = 1.6378V
// Para Vin = +6V -> V_adc = 3.2756V
// Factor de escala = 12V / 3.2756V = 3.6635 V/V
const float V_REF = 3.3f;
const float V_OFFSET_ARD = 1.6378f;
const float ESCALA_V = 3.6635f;

inline float adcAVoltaje(int adc) {
  float v_ard = (adc / 4095.0f) * V_REF;
  return (v_ard - V_OFFSET_ARD) * ESCALA_V;
}

inline int voltajeADac(float v) {
  float v_ard = (v / ESCALA_V) + V_OFFSET_ARD;
  int dac = (int)((v_ard / V_REF) * 4095.0f + 0.5f);
  if (dac < 0) dac = 0;
  else if (dac > 4095) dac = 4095;
  return dac;
}

// Parámetros de muestreo
const float DT_FILTRO = 0.001f; // 1 ms = 1000 Hz

// Estado de Filtros
volatile int tipo_filtro = 0; // 0=sin filtro, 1=pasabajas, 2=pasaaltos, 3=pasabanda
volatile float fc_filtro = 20.0f;
volatile int muestreo_activo = 1;

// Coeficientes precalculados
float alfa_lp = 1.0f;
float alfa_hp = 1.0f;
float alfa_bp_lp = 1.0f;
float alfa_bp_hp = 1.0f;

// Memorias de estado del filtro
float x_prev = 0.0f;
float y_lp_prev = 0.0f;
float y_hp_prev = 0.0f;
float y_bp_lp_prev = 0.0f;
float y_bp_hp_prev = 0.0f;
float w_bp_prev = 0.0f;

void recalcularCoeficientes() {
  float fc = fc_filtro;
  if (fc < 0.5f) fc = 0.5f;
  if (fc > 400.0f) fc = 400.0f;
  
  // Pasabajas IIR: y[n] = alfa*x[n] + (1-alfa)*y[n-1]
  float rc_lp = 1.0f / (2.0f * 3.14159265f * fc);
  alfa_lp = DT_FILTRO / (rc_lp + DT_FILTRO);
  
  // Pasaaltos IIR: y[n] = alfa*(y[n-1] + x[n] - x[n-1])
  float rc_hp = 1.0f / (2.0f * 3.14159265f * fc);
  alfa_hp = rc_hp / (rc_hp + DT_FILTRO);
  
  // Pasabanda: cascada LPF (fc_alta) y HPF (fc_baja)
  float bw = (fc * 0.3f < 4.0f) ? 4.0f : (fc * 0.3f);
  float fc_alta = fc + (bw / 2.0f);
  if (fc_alta > 450.0f) fc_alta = 450.0f;
  float rc_bp_lp = 1.0f / (2.0f * 3.14159265f * fc_alta);
  alfa_bp_lp = DT_FILTRO / (rc_bp_lp + DT_FILTRO);
  
  float fc_baja = fc - (bw / 2.0f);
  if (fc_baja < 0.5f) fc_baja = 0.5f;
  float rc_bp_hp = 1.0f / (2.0f * 3.14159265f * fc_baja);
  alfa_bp_hp = rc_bp_hp / (rc_bp_hp + DT_FILTRO);
  
  // Limpiar estados para evitar transitorios
  y_lp_prev = 0.0f;
  y_hp_prev = 0.0f;
  y_bp_lp_prev = 0.0f;
  y_bp_hp_prev = 0.0f;
  w_bp_prev = 0.0f;
  x_prev = 0.0f;
}

inline float aplicarFiltro(float x_act) {
  float salida = x_act;
  
  if (tipo_filtro == 1) { // Pasabajas
    salida = (alfa_lp * x_act) + ((1.0f - alfa_lp) * y_lp_prev);
    y_lp_prev = salida;
  } else if (tipo_filtro == 2) { // Pasaaltos
    salida = alfa_hp * (y_hp_prev + x_act - x_prev);
    y_hp_prev = salida;
  } else if (tipo_filtro == 3) { // Pasabanda
    // Etapa 1: LPF -> w
    float w = (alfa_bp_lp * x_act) + ((1.0f - alfa_bp_lp) * y_bp_lp_prev);
    y_bp_lp_prev = w;
    // Etapa 2: HPF aplicado sobre w
    salida = alfa_bp_hp * (y_bp_hp_prev + w - w_bp_prev);
    y_bp_hp_prev = salida;
    w_bp_prev = w;
  }
  
  x_prev = x_act;
  return salida;
}

// BUFFER CIRCULAR LOCK-FREE PARA TELEMETRÍA
// Almacena hasta 1024 pares (vin_mv, vout_mv) como int16_t
// A 1000 Hz, 1024 muestras = > 1 segundo de amortiguación
const int RING_BUFFER_SIZE = 1024;
int16_t ring_vin[RING_BUFFER_SIZE];
int16_t ring_vout[RING_BUFFER_SIZE];
volatile uint32_t ring_head = 0; // Leído por RPC
volatile uint32_t ring_tail = 0; // Escrito por Sampler Thread

inline void ring_push(int16_t vin_mv, int16_t vout_mv) {
  uint32_t next_tail = (ring_tail + 1) & (RING_BUFFER_SIZE - 1);
  if (next_tail != ring_head) {
    ring_vin[ring_tail] = vin_mv;
    ring_vout[ring_tail] = vout_mv;
    ring_tail = next_tail;
  }
}

// Retorna hasta 64 muestras (128 int16 = 256 bytes, seguro dentro de 1024 DECODER_BUFFER_SIZE)
const int MAX_RPC_SAMPLES = 64;

std::vector<int16_t> leerTelemetria() {
  std::vector<int16_t> datos;
  uint32_t head = ring_head;
  uint32_t tail = ring_tail;
  
  uint32_t disponibles = (tail >= head) ? (tail - head) : (RING_BUFFER_SIZE - head + tail);
  uint32_t a_leer = (disponibles > MAX_RPC_SAMPLES) ? MAX_RPC_SAMPLES : disponibles;
  
  if (a_leer > 0) {
    datos.reserve(a_leer * 2);
    for (uint32_t i = 0; i < a_leer; i++) {
      datos.push_back(ring_vin[head]);
      datos.push_back(ring_vout[head]);
      head = (head + 1) & (RING_BUFFER_SIZE - 1);
    }
    ring_head = head;
  }
  return datos;
}

int configurarFiltro(int tipo, double fc) {
  tipo_filtro = tipo;
  fc_filtro = (float)fc;
  recalcularCoeficientes();
  return 1;
}

int controlMuestreo(int activo) {
  muestreo_activo = activo;
  return muestreo_activo;
}

// Retrocompatibilidad
int leerSenal() {
  return analogRead(pinADC);
}

int escribirDAC(int valor) {
  analogWrite(PIN_DAC, valor);
  return 1;
}

// HILO DE MUESTREO EN TIEMPO REAL (ZEPHYR RTOS)
#define SAMPLER_STACK_SIZE 2048
#define SAMPLER_PRIO 1 // Mayor prioridad que Bridge (prio 5)

K_THREAD_STACK_DEFINE(sampler_stack_area, SAMPLER_STACK_SIZE);
struct k_thread sampler_thread;

void sampler_thread_entry(void *, void *, void *) {
  int64_t next_tick = k_uptime_get();
  
  while (1) {
    next_tick += 1; // 1 ms exacto (1000 Hz)
    int64_t now = k_uptime_get();
    if (next_tick <= now) {
      next_tick = now + 1;
    }
    k_sleep(K_TIMEOUT_ABS_MS(next_tick));
    
    if (!muestreo_activo) continue;
    
    int adc = analogRead(pinADC);
    float vin = adcAVoltaje(adc);
    float vout = aplicarFiltro(vin);
    int dac = voltajeADac(vout);
    analogWrite(PIN_DAC, dac);
    
    int16_t vin_mv = (int16_t)(vin * 1000.0f);
    int16_t vout_mv = (int16_t)(vout * 1000.0f);
    ring_push(vin_mv, vout_mv);
  }
}

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  
  recalcularCoeficientes();
  
  // Iniciar hilo de muestreo en tiempo real
  k_thread_create(&sampler_thread, sampler_stack_area,
                  K_THREAD_STACK_SIZEOF(sampler_stack_area),
                  sampler_thread_entry, NULL, NULL, NULL,
                  SAMPLER_PRIO, 0, K_NO_WAIT);
  k_thread_name_set(&sampler_thread, "dsp_sampler");
  
  Bridge.begin();
  Bridge.provide("leerA4", leerSenal);
  Bridge.provide("escribirDAC", escribirDAC);
  Bridge.provide("configurarFiltro", configurarFiltro);
  Bridge.provide("controlMuestreo", controlMuestreo);
  Bridge.provide("leerTelemetria", leerTelemetria);
}

void loop() {
  // El lazo de muestreo corre en su propio hilo RTOS a 1000 Hz exactos.
  k_sleep(K_MSEC(100));
}
