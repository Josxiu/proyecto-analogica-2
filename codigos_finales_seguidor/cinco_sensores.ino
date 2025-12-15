#include <Arduino.h>

// =========================
//  CONFIGURACIÓN DE PINES
// =========================
const int IN1 = 15; const int IN2 = 7;  const int ENA = 16; 
const int IN3 = 8;  const int IN4 = 18; const int ENB = 17; 

// --- SENSORES ANALÓGICOS (CENTRALES) ---
const int S_LEFT = 4;
const int S_CENTER = 5;
const int S_RIGHT = 6;

// --- NUEVOS SENSORES DIGITALES (EXTREMOS) ---
const int S_EXT_IZQ = 40; // Ala Izquierda
const int S_EXT_DER = 39; // Ala Derecha

// PWM Config
const int freqPWM = 5000;
const int resolucion = 8;

// =========================
//  CALIBRACIÓN ANALÓGICA
// =========================
// Tus valores calibrados para los 3 del centro
int umbralL = 1200;
int umbralC = 1200;
int umbralR = 1500; 

// =========================
//  TUNING (AJUSTES)
// =========================
int velMax = 170;       

// PID
float Kp = 55.0; 
float Kd = 40.0; 

// FRENADO
float factorFreno = 0.9; 

// ZONA MUERTA
const int MIN_POWER = 115; 

// =========================
//  VARIABLES
// =========================
int lastError = 0;       
int lastSeenSide = 1;    

// Estados de Recuperación
enum RecStage { REC_IDLE = 0, REC_BRAKE, REC_REVERSE, REC_SCAN_1, REC_SCAN_2, REC_CENTER };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;

// =========================
//  FUNCIONES MOTOR
// =========================
int corregirZonaMuerta(int pwm) {
  if (abs(pwm) > 0 && abs(pwm) < MIN_POWER) {
    if (pwm > 0) return MIN_POWER;
    else return -MIN_POWER;
  }
  return pwm;
}

void setMotor(int speedLeft, int speedRight) {
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);
  
  speedLeft = corregirZonaMuerta(speedLeft);
  speedRight = corregirZonaMuerta(speedRight);

  if (speedLeft >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(ENB, speedLeft);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(ENB, abs(speedLeft));
  }

  if (speedRight >= 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(ENA, speedRight);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(ENA, abs(speedRight));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Configurar los nuevos sensores como ENTRADA
  pinMode(S_EXT_IZQ, INPUT);
  pinMode(S_EXT_DER, INPUT);

  ledcAttach(ENA, freqPWM, resolucion);
  ledcAttach(ENB, freqPWM, resolucion);
  Serial.println("SISTEMA 5 SENSORES LISTO");
  delay(2000);
}

void loop() {
  // 1. LECTURA DE SENSORES
  
  // Analógicos (Lógica: Alto es Negro)
  bool L = analogRead(S_LEFT) > umbralL;
  bool C = analogRead(S_CENTER) > umbralC;
  bool R = analogRead(S_RIGHT) > umbralR;

  // Digitales Nuevos (Lógica: LOW/0 es Negro)
  // Usamos '!' para invertir: Si lee 0 (false), lo volvemos true (Detectado)
  bool XL = !digitalRead(S_EXT_IZQ); 
  bool XR = !digitalRead(S_EXT_DER); 

  int error = 0;
  bool lineaDetectada = false;

  // 2. LÓGICA DE 5 SENSORES (Prioridad a los extremos)
  
  // CASOS EXTREMOS (Curvas cerradas) -> Error 3 o -3
  if (XL) { 
    error = -3; // Muy a la izquierda
    lineaDetectada = true; 
    lastSeenSide = -1; 
  }
  else if (XR) { 
    error = 3;  // Muy a la derecha
    lineaDetectada = true; 
    lastSeenSide = 1; 
  }
  // CASOS NORMALES (Rectas y curvas suaves) -> Error -2 a 2
  else if (L && !C && !R)      { error = -2; lineaDetectada = true; lastSeenSide = -1; }
  else if (L && C && !R)  { error = -1; lineaDetectada = true; lastSeenSide = -1; }
  else if (!L && C && !R) { error = 0;  lineaDetectada = true; } 
  else if (!L && C && R)  { error = 1;  lineaDetectada = true; lastSeenSide = 1; }
  else if (!L && !C && R) { error = 2;  lineaDetectada = true; lastSeenSide = 1; }
  else if (L && C && R)   { error = 0;  lineaDetectada = true; } // Cruce o línea gorda

  if (lineaDetectada) {
    recStage = REC_IDLE; 
    
    int P = error;
    int D = error - lastError;
    int correccionPID = (Kp * P) + (Kd * D);
    lastError = error;

    // FRENADO ADAPTATIVO
    // Nota: Como ahora el error puede ser 3, el frenado será aún más fuerte en curvas extremas.
    int velocidadBase = velMax - (abs(correccionPID) * factorFreno);
    
    if (velocidadBase < MIN_POWER) velocidadBase = MIN_POWER;

    int velIzq = velocidadBase - correccionPID;
    int velDer = velocidadBase + correccionPID;

    setMotor(velIzq, velDer);

  } else {
    manejarRecuperacion();
  }
  delay(1); 
}

// =========================
//  RECUPERACIÓN (Mantenemos la que funciona)
// =========================
void manejarRecuperacion() {
  unsigned long now = millis();
  int velScan = 130;      
  int velReverse = -150; 

  switch (recStage) {
    case REC_IDLE:
      recStage = REC_BRAKE;
      recT0 = now;
      setMotor(-255, -255); 
      break;

    case REC_BRAKE:
      if (now - recT0 > 80) { recStage = REC_REVERSE; recT0 = now; }
      break;

    case REC_REVERSE:
      setMotor(velReverse, velReverse);
      // Solo 120ms
      if (now - recT0 > 120) { recStage = REC_SCAN_1; recT0 = now; }
      break;

    case REC_SCAN_1:
      if (lastSeenSide == -1) setMotor(-velScan, velScan); else setMotor(velScan, -velScan); 
      if (now - recT0 > 400) { recStage = REC_SCAN_2; recT0 = now; }
      break;

    case REC_SCAN_2:
      if (lastSeenSide == -1) setMotor(velScan, -velScan); else setMotor(-velScan, velScan); 
      if (now - recT0 > 800) { recStage = REC_CENTER; recT0 = now; }
      break;

    case REC_CENTER:
       setMotor(0,0); 
       break;
  }
}