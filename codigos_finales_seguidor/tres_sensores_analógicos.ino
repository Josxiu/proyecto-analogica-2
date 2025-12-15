#include <Arduino.h>

// =========================
//  CONFIGURACIÓN DE PINES
// =========================
const int IN1 = 15; const int IN2 = 7;  const int ENA = 16; 
const int IN3 = 8;  const int IN4 = 18; const int ENB = 17; 

const int S_LEFT = 4;
const int S_CENTER = 5;
const int S_RIGHT = 6;

// PWM Config
const int freqPWM = 5000;
const int resolucion = 8;

// =========================
//  CALIBRACIÓN (LO MÁS IMPORTANTE)
// =========================
// Calculamos el punto medio exacto para cada sensor según tus datos.
// Fórmula: (Blanco + Negro) / 2
// Izq/Centro: (60 + 3000)/2 = ~1500
// Derecho:    (60 + 3900)/2 = ~2000  <-- ¡Este era el culpable!

int umbralL = 1200;
int umbralC = 1200;
int umbralR = 1500; // Umbral más alto para el sensor más sensible

// =========================
//  TUNING (AJUSTES)
// =========================
int velMax = 170;       

// PID DIGITAL (Volvemos a los valores grandes)
float Kp = 55.0; // Subí un poco para más fuerza en curvas
float Kd = 40.0; // Amortiguación fuerte

// FRENADO EN CURVAS (0.0 = nada, 1.0 = frena total)
float factorFreno = 0.9; // Agresivo: queremos que frene MUCHO en curvas cerradas

// ZONA MUERTA (Para que no pite y se quede quieto)
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
  // Si el motor debe moverse pero la potencia es muy baja, le damos un empujón
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
  ledcAttach(ENA, freqPWM, resolucion);
  ledcAttach(ENB, freqPWM, resolucion);
  Serial.println("MODO DIGITAL CALIBRADO LISTO");
  delay(2000);
}

void loop() {
  // 1. Lectura con UMBRALES INDIVIDUALES
  // Esto corrige que el robot tire más para un lado
  bool L = analogRead(S_LEFT) > umbralL;
  bool C = analogRead(S_CENTER) > umbralC;
  bool R = analogRead(S_RIGHT) > umbralR;

  int error = 0;
  bool lineaDetectada = false;

  // 2. Lógica Digital (La que mejor te funcionó)
  if (L && !C && !R)      { error = -2; lineaDetectada = true; lastSeenSide = -1; }
  else if (L && C && !R)  { error = -1; lineaDetectada = true; lastSeenSide = -1; }
  else if (!L && C && !R) { error = 0;  lineaDetectada = true; } 
  else if (!L && C && R)  { error = 1;  lineaDetectada = true; lastSeenSide = 1; }
  else if (!L && !C && R) { error = 2;  lineaDetectada = true; lastSeenSide = 1; }
  else if (L && C && R)   { error = 0;  lineaDetectada = true; } 

  if (lineaDetectada) {
    recStage = REC_IDLE; 
    
    int P = error;
    int D = error - lastError;
    int correccionPID = (Kp * P) + (Kd * D);
    lastError = error;

    // --- FRENADO ADAPTATIVO "TANQUE" ---
    // Si la corrección es fuerte (curva cerrada), bajamos la velocidad base
    // Con factorFreno 0.9, si el PID pide 100, la velocidad base baja 90.
    int velocidadBase = velMax - (abs(correccionPID) * factorFreno);
    
    // Aseguramos que nunca baje de MIN_POWER para que no se tranque
    if (velocidadBase < MIN_POWER) velocidadBase = MIN_POWER;

    int velIzq = velocidadBase - correccionPID;
    int velDer = velocidadBase + correccionPID;

    setMotor(velIzq, velDer);

  } else {
    manejarRecuperacion();
  }
  delay(2); 
}

// =========================
//  RECUPERACIÓN HÍBRIDA (Retroceso + Búsqueda Simple)
// =========================
void manejarRecuperacion() {
  unsigned long now = millis();
  int velScan = 130;      
  int velReverse = -150; 

  switch (recStage) {
    case REC_IDLE:
      recStage = REC_BRAKE;
      recT0 = now;
      setMotor(-255, -255); // Frenado pánico
      break;

    case REC_BRAKE:
      // Frenamos rápido (80ms)
      if (now - recT0 > 80) { 
        recStage = REC_REVERSE;
        recT0 = now;
      }
      break;

    case REC_REVERSE:
      // "Golpe atrás" táctico para volver a la pista
      setMotor(velReverse, velReverse);
      // Solo 120ms, suficiente para corregir el "overshoot"
      if (now - recT0 > 120) {
        recStage = REC_SCAN_1;
        recT0 = now;
      }
      break;

    case REC_SCAN_1:
      // Girar al lado donde la vimos por última vez
      if (lastSeenSide == -1) setMotor(-velScan, velScan); 
      else setMotor(velScan, -velScan); 
      
      // Tiempo moderado (400ms)
      if (now - recT0 > 400) { 
        recStage = REC_SCAN_2; 
        recT0 = now;
      }
      break;

    case REC_SCAN_2:
      // Girar al lado contrario (porsiaca)
      if (lastSeenSide == -1) setMotor(velScan, -velScan); 
      else setMotor(-velScan, velScan); 
      
      // Tiempo doble para cruzar (800ms)
      if (now - recT0 > 800) { 
        recStage = REC_CENTER; 
        recT0 = now;
      }
      break;

    case REC_CENTER:
       setMotor(0,0); // Rendirse
       break;
  }
}