#include <Arduino_RouterBridge.h>
#include <vector>
#include <math.h>

#define PIN_DAC DAC0
const int pinADC = A4;

// Conversión de niveles de tensión (+/- 6V <-> 0..3.3V)
const float V_REF = 3.3f;
const float ESCALA_V = 3.63636f; // Factor Level Shifter (+/-6V / 1.65V)

inline float adcAVoltaje(int adc) {
  float v_ard = (adc / 4095.0f) * V_REF;
  return (v_ard - 1.65f) * ESCALA_V;
}

inline int voltajeADac(float v) {
  float v_ard = (v / ESCALA_V) + 1.65f;
  int dac = (int)((v_ard / V_REF) * 4095.0f);
  if (dac < 0) dac = 0;
  else if (dac > 4095) dac = 4095;
  return dac;
}

// Frecuencia de muestreo del lazo hardware: 1000 Hz (1 ms)
const unsigned long PERIODO_MUESTREO_US = 1000;
const float DT_FILTRO = 0.001f;

// Estado del filtro digital
int tipo_filtro = 0; // 0=ninguno, 1=pasabajas, 2=pasaaltos, 3=pasabanda
float fc_filtro = 5.0f;

// Coeficientes precalculados para máxima velocidad
float alfa_lp = 1.0f;
float alfa_hp = 1.0f;
float alfa_bp_lp = 1.0f;
float alfa_bp_hp = 1.0f;

// Memorias de diferencias IIR
float x_prev = 0.0f;
float y_lp_prev = 0.0f;
float y_hp_prev = 0.0f;
float y_bp_lp_prev = 0.0f;
float y_bp_hp_prev = 0.0f;

void recalcularCoeficientes() {
  float fc = (fc_filtro < 0.1f) ? 0.1f : fc_filtro;
  
  // Pasabajas
  float rc_lp = 1.0f / (2.0f * 3.14159265f * fc);
  alfa_lp = DT_FILTRO / (rc_lp + DT_FILTRO);
  
  // Pasaaltos
  float rc_hp = 1.0f / (2.0f * 3.14159265f * fc);
  alfa_hp = rc_hp / (rc_hp + DT_FILTRO);
  
  // Pasabanda
  float fc_alta = fc + 2.0f;
  float rc_bp_lp = 1.0f / (2.0f * 3.14159265f * fc_alta);
  alfa_bp_lp = DT_FILTRO / (rc_bp_lp + DT_FILTRO);
  
  float fc_baja = (fc - 2.0f > 0.1f) ? (fc - 2.0f) : 0.1f;
  float rc_bp_hp = 1.0f / (2.0f * 3.14159265f * fc_baja);
  alfa_bp_hp = rc_bp_hp / (rc_bp_hp + DT_FILTRO);
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
    float y_lp = (alfa_bp_lp * x_act) + ((1.0f - alfa_bp_lp) * y_bp_lp_prev);
    y_bp_lp_prev = y_lp;
    salida = alfa_bp_hp * (y_bp_hp_prev + y_lp - x_prev);
    y_bp_hp_prev = salida;
  }
  
  x_prev = x_act;
  return salida;
}

// Telemetría hacia Python a 200 Hz (1000 Hz / 5)
const int DECIMACION_TELEMETRIA = 5;
int contador_decimacion = 0;

const int CAPACIDAD_TELEMETRIA = 40; // hasta 20 pares (v_in, v_out)
float buffer_telemetria[CAPACIDAD_TELEMETRIA];
volatile int tam_telemetria = 0;

void guardarTelemetria(float vin, float vout) {
  if (tam_telemetria + 2 <= CAPACIDAD_TELEMETRIA) {
    buffer_telemetria[tam_telemetria++] = vin;
    buffer_telemetria[tam_telemetria++] = vout;
  }
}

// RPC: Llamado periódicamente por Python (~20-25 Hz)
std::vector<float> leerTelemetria() {
  int cant = tam_telemetria;
  std::vector<float> datos(cant);
  for (int i = 0; i < cant; i++) {
    datos[i] = buffer_telemetria[i];
  }
  tam_telemetria = 0; // vaciar buffer
  return datos;
}

// RPC: Configurar filtro desde la Web / Python
int configurarFiltro(int tipo, float fc) {
  tipo_filtro = tipo;
  fc_filtro = fc;
  recalcularCoeficientes();
  return 1;
}

// RPC retrocompatibilidad
int leerSenal() {
  return analogRead(pinADC);
}

int escribirDAC(int valor) {
  analogWrite(PIN_DAC, valor);
  return 1;
}

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  
  recalcularCoeficientes();
  
  Bridge.begin();
  Bridge.provide("leerA4", leerSenal);
  Bridge.provide("escribirDAC", escribirDAC);
  Bridge.provide("configurarFiltro", configurarFiltro);
  Bridge.provide("leerTelemetria", leerTelemetria);
}

unsigned long t_ultimo_muestreo = 0;

void loop() {
  unsigned long t_actual = micros();
  if (t_actual - t_ultimo_muestreo >= PERIODO_MUESTREO_US) {
    t_ultimo_muestreo += PERIODO_MUESTREO_US;
    if (t_actual - t_ultimo_muestreo > 5 * PERIODO_MUESTREO_US) {
      t_ultimo_muestreo = t_actual;
    }
    
    int adc = analogRead(pinADC);
    float vin = adcAVoltaje(adc);
    float vout = aplicarFiltro(vin);
    int dac = voltajeADac(vout);
    analogWrite(PIN_DAC, dac);
    
    contador_decimacion++;
    if (contador_decimacion >= DECIMACION_TELEMETRIA) {
      contador_decimacion = 0;
      guardarTelemetria(vin, vout);
    }
  }
}