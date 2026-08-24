#include <Arduino_RouterBridge.h>

#define PIN_DAC DAC0
const int pinADC = A4; 

// Función para leer
int leerSenal() {
  return analogRead(pinADC);
}

// Función para escribir - Recibe el valor procesado desde Python
int escribirDAC(int valor) {
  analogWrite(PIN_DAC, valor);
  return 1; // Retornamos 1 para confirmar éxito al bridge RPC
}

void setup() {
  // Configuramos 12 bits para entrada y salida
  analogReadResolution(12);
  analogWriteResolution(12);
  
  Bridge.begin();
  
  // Registramos las funciones en el RouterBridge
  Bridge.provide("leerA4", leerSenal);
  Bridge.provide("escribirDAC", escribirDAC);
}

void loop() {
  // El puente de enrutamiento maneja las llamadas RPC automáticamente en segundo plano
}