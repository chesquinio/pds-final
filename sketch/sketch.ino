#include <Arduino_RouterBridge.h>

#define PIN_DAC DAC0
const int pinADC = A4; 

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
  
  Bridge.begin();
  Bridge.provide("leerA4", leerSenal);
  Bridge.provide("escribirDAC", escribirDAC);
}

void loop() {
}