#include <Arduino.h>

// ==========================================
//   CONFIGURACIÓN DE PINES (TUS PINES ORIGINALES)
// ==========================================
// Motor A (TU RUEDA IZQUIERDA)
const int ENA = 46;
const int IN1 = 45;
const int IN2 = 40;

// Motor B (TU RUEDA DERECHA)
const int ENB = 39;
const int IN3 = 36;
const int IN4 = 35;

// Sensores Analógicos
const int S_LEFT = 4;
const int S_CENTER = 5;
const int S_RIGHT = 6;

// Configuración PWM ESP32
const int canalA = 0; // Controla Motor Izquierdo
const int canalB = 1; // Controla Motor Derecho
const int freqPWM = 5000;
const int resolucion = 8;

// ==========================================
//   VARIABLES PID
// ==========================================
float Kp = 0.18;   // Proporcional
float Kd = 1.5;    // Derivativo
float Ki = 0.0;    // Integral

// Velocidades
int velocidadBase = 170;  
int velocidadMax = 240;   

// Variables internas PID
int error = 0;
int lastError = 0;
int P = 0, I = 0, D = 0;
int pidValue = 0;

// Calibración (Valores extremos iniciales)
int minL = 4095, maxL = 0;
int minC = 4095, maxC = 0;
int minR = 4095, maxR = 0;

// ==========================================
//   CONTROL MOTORES (LÓGICA ADAPTADA)
// ==========================================
void setMotor(int speedLeft, int speedRight) {
  // Limitamos la velocidad para no pasarnos de rosca
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  // --- MOTOR A (IZQUIERDO) ---
  // Tu lógica original: Adelante = IN1 LOW, IN2 HIGH
  if (speedLeft >= 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
    ledcWrite(canalA, speedLeft);
  } else {
    // Atrás
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    ledcWrite(canalA, abs(speedLeft));
  }

  // --- MOTOR B (DERECHO) ---
  // Tu lógica original: Adelante = IN3 HIGH, IN4 LOW
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    ledcWrite(canalB, speedRight);
  } else {
    // Atrás
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
    ledcWrite(canalB, abs(speedRight));
  }
}

// ==========================================
//   CALIBRACIÓN
// ==========================================
void calibrarSensores() {
  Serial.println("Calibrando...");
  // Girar sobre su eje (Izq atrás, Der adelante)
  setMotor(-100, 100); 
  
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) { 
    int L = analogRead(S_LEFT);
    int C = analogRead(S_CENTER);
    int R = analogRead(S_RIGHT);

    // Guardar máximos y mínimos
    if (L < minL) minL = L; if (L > maxL) maxL = L;
    if (C < minC) minC = C; if (C > maxC) maxC = C;
    if (R < minR) minR = R; if (R > maxR) maxR = R;
  }
  setMotor(0, 0); // Detener
}

// ==========================================
//   SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  ledcSetup(canalA, freqPWM, resolucion);
  ledcSetup(canalB, freqPWM, resolucion);
  ledcAttachPin(ENA, canalA);
  ledcAttachPin(ENB, canalB);

  delay(1000);
  calibrarSensores(); // ¡Asegúrate de ponerlo en la línea aquí!
  delay(1000);
}

// ==========================================
//   LOOP PID
// ==========================================
void loop() {
  // 1. Lectura y Normalización (0 a 1000)
  // Asumimos que BLANCO es valor ALTO y NEGRO es valor BAJO (típico infrarrojo analógico)
  // Si tu sensor es al revés, invierte el map: map(val, min, max, 0, 1000)
  
  // Lógica basada en tu código: "rawL < umbralNegro" -> Negro da valores bajos.
  // Por tanto: MinValue = Negro, MaxValue = Blanco.
  // Queremos que Negro sea 1000 para la matemática del error.
  int valL = map(analogRead(S_LEFT), minL, maxL, 1000, 0);
  int valC = map(analogRead(S_CENTER), minC, maxC, 1000, 0); 
  int valR = map(analogRead(S_RIGHT), minR, maxR, 1000, 0);

  valL = constrain(valL, 0, 1000);
  valC = constrain(valC, 0, 1000);
  valR = constrain(valR, 0, 1000);

  // 2. Cálculo del Error
  // Positivo = Linea a la derecha (Robot debe girar derecha)
  // Negativo = Linea a la izquierda (Robot debe girar izquierda)
  error = valR - valL;

  // Lógica para curvas cerradas (Si el del centro deja de ver la línea)
  if (valC < 200) { 
     // Si el centro perdió la línea, confiamos más en los extremos
     if (valR > valL) error = 1500; // Muy a la derecha
     else error = -1500;            // Muy a la izquierda
  }

  // 3. PID
  P = error;
  D = error - lastError;
  pidValue = (Kp * P) + (Kd * D);
  lastError = error;

  // 4. Aplicar a motores
  // Si error es positivo (línea a la derecha), pidValue será positivo.
  // Queremos ir a la derecha -> Aumentar Izq, Disminuir Der.
  int motorIzq = velocidadBase + pidValue;
  int motorDer = velocidadBase - pidValue;

  setMotor(motorIzq, motorDer);
}
