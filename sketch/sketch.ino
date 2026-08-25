#include <Arduino_RouterBridge.h>
#include <Arduino_LED_Matrix.h>
#include "frames.h" // Importamos las ondas generadas matemáticamente

#define PIN_DAC DAC0
const int pinADC = A4; 

Arduino_LED_Matrix matrix; 

int leerSenal() {
  return analogRead(pinADC);
}

int escribirDAC(int valor) {
  analogWrite(PIN_DAC, valor);
  return 1;
}

// Python llama a esta función con 0, 1, 2 o 3
int dibujarFiltro(int tipo) {
  switch(tipo) {
    case 0: matrix.loadSequence(anim_original); break;
    case 1: matrix.loadSequence(anim_pasabajas); break;
    case 2: matrix.loadSequence(anim_pasaaltos); break;
    case 3: matrix.loadSequence(anim_pasabanda); break;
    default: matrix.loadSequence(anim_original); break;
  }
  return 1;
}

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  
  matrix.begin();
  matrix.loadSequence(anim_original);
  
  Bridge.begin();
  Bridge.provide("leerA4", leerSenal);
  Bridge.provide("escribirDAC", escribirDAC);
  Bridge.provide("dibujarFiltro", dibujarFiltro);
}

void loop() {
  matrix.playSequence(true); 
}