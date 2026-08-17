#include <Arduino_RouterBridge.h>

const int pinADC = A4; 

int leerSenal() {
  return analogRead(pinADC);
}

void setup() {
  // Configuración del ADC a máxima resolución (0-4095)
  analogReadResolution(12);

  Bridge.begin();
  Bridge.provide("leerA4", leerSenal);
}

void loop() {
  // El loop se mantiene libre para el RouterBridge
  delay(100);
}