#include <Arduino.h>

// =========================
//  Definición de Pines
// =========================

// Sensores Analógicos (Centrales)
const int S_LEFT = 4;
const int S_CENTER = 5;
const int S_RIGHT = 6;

// Sensores Digitales (Extremos)
const int S_EXT_IZQ = 40;
const int S_EXT_DER = 39;

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines (INPUT)
  // Analógicos
  pinMode(S_LEFT, INPUT);
  pinMode(S_CENTER, INPUT);
  pinMode(S_RIGHT, INPUT);
  
  // Digitales
  pinMode(S_EXT_IZQ, INPUT);
  pinMode(S_EXT_DER, INPUT);

  Serial.println("=== TEST DE 5 SENSORES (Analógicos + Digitales) ===");
  Serial.println("Orden: [Ext.Izq] - [Izq] - [Cen] - [Der] - [Ext.Der]");
  delay(2000);
}

void loop() {
  // 1. Leer Sensores Digitales (Devuelven 0 o 1)
  int valExtIzq = digitalRead(S_EXT_IZQ);
  int valExtDer = digitalRead(S_EXT_DER);

  // 2. Leer Sensores Analógicos (Devuelven 0 a 4095 en ESP32)
  int valL = analogRead(S_LEFT);
  int valC = analogRead(S_CENTER);
  int valR = analogRead(S_RIGHT);

  // 3. Imprimir en una sola línea ordenada visualmente
  // Formato: ExtIzq | Izq | Cen | Der | ExtDer
  
  Serial.print("E_IZQ: ");
  Serial.print(valExtIzq); // 0 o 1
  
  Serial.print("\t| L: ");
  Serial.print(valL);      // 0-4095
  
  Serial.print("\t| C: ");
  Serial.print(valC);      // 0-4095
  
  Serial.print("\t| R: ");
  Serial.print(valR);      // 0-4095
  
  Serial.print("\t| E_DER: ");
  Serial.print(valExtDer); // 0 o 1

  Serial.println(); // Salto de línea final

  delay(200); // Pausa para legibilidad
}
