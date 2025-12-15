#include <Arduino.h>

// =========================
//  CONFIGURACIÓN DE PINES
// =========================
const int IN1 = 15; const int IN2 = 7;  const int ENA = 16; 
const int IN3 = 8;  const int IN4 = 18; const int ENB = 17; 

// Sensores
const int S_LEFT = 4;
const int S_CENTER = 5;
const int S_RIGHT = 6;
const int S_EXT_IZQ = 40; 
const int S_EXT_DER = 39; 

// PWM
const int freqPWM = 5000;
const int resolucion = 8;

// =========================
//  CALIBRACIÓN (Tus valores)
// =========================
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
float factorFreno = 0.9; 
const int MIN_POWER = 115; 

// --- AJUSTES DE RECUPERACIÓN (Basados en tu video) ---
// Para hacer el arco: Una rueda empuja fuerte, la otra frena un poco
int velGiroRapido = 150; 
int velGiroLento = -50;  // Un poquito hacia atrás para pivotar mejor (Efecto compás)

// AUMENTÉ ESTOS TIEMPOS:
const int T_REVERSA = 150; // Un paso atrás para alejarse del borde
const int T_BARRIDO_1 = 500; // Barrido inicial
const int T_BARRIDO_2 = 900; // Barrido de regreso (más largo)

// =========================
//  VARIABLES
// =========================
int lastError = 0;       
int lastSeenSide = 1; 

enum RecStage { REC_IDLE = 0, REC_BRAKE, REC_REVERSE, REC_SCAN_1, REC_SCAN_2, REC_CENTER };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;

// =========================
//  FUNCIONES
// =========================
int corregirZonaMuerta(int pwm) {
  if (abs(pwm) > 0 && abs(pwm) < MIN_POWER) {
    return (pwm > 0) ? MIN_POWER : -MIN_POWER;
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
  pinMode(S_EXT_IZQ, INPUT);
  pinMode(S_EXT_DER, INPUT);
  ledcAttach(ENA, freqPWM, resolucion);
  ledcAttach(ENB, freqPWM, resolucion);
  Serial.println("LISTO PARA LA PISTA");
  delay(2000);
}

void loop() {
  bool L = analogRead(S_LEFT) > umbralL;
  bool C = analogRead(S_CENTER) > umbralC;
  bool R = analogRead(S_RIGHT) > umbralR;
  bool XL = !digitalRead(S_EXT_IZQ); 
  bool XR = !digitalRead(S_EXT_DER); 

  int error = 0;
  bool lineaDetectada = false;

  // Lógica Digital -3 a 3
  if (XL) { error = -3; lineaDetectada = true; lastSeenSide = -1; }
  else if (XR) { error = 3;  lineaDetectada = true; lastSeenSide = 1; }
  else if (L && !C && !R) { error = -2; lineaDetectada = true; lastSeenSide = -1; }
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
//  RECUPERACIÓN EN "S" AMPLIADA
// =========================
void manejarRecuperacion() {
  unsigned long now = millis();

  switch (recStage) {
    case REC_IDLE:
      recStage = REC_BRAKE;
      recT0 = now;
      setMotor(-255, -255); // Frenazo seco
      break;

    case REC_BRAKE:
      if (now - recT0 > 100) { recStage = REC_REVERSE; recT0 = now; }
      break;

    case REC_REVERSE:
      // Retrocedemos un poco para despegar la nariz del obstáculo
      setMotor(-150, -150);
      if (now - recT0 > T_REVERSA) { recStage = REC_SCAN_1; recT0 = now; }
      break;

    case REC_SCAN_1:
      // FASE 1: BARRIDO AL LADO DONDE SE FUE LA LÍNEA
      // Truco del video: Hacemos que gire más "abierto".
      // Una rueda va RÁPIDO ADELANTE, la otra va LENTO ATRÁS.
      
      if (lastSeenSide == -1) {
         // Se fue a la Izquierda -> Girar Izquierda
         // Motor Izq (-50), Motor Der (150) -> Pivote desplazado
         setMotor(velGiroLento, velGiroRapido); 
      } else {
         // Se fue a la Derecha -> Girar Derecha
         setMotor(velGiroRapido, velGiroLento); 
      }
      
      if (now - recT0 > T_BARRIDO_1) { 
        recStage = REC_SCAN_2; 
        recT0 = now; 
      }
      break;

    case REC_SCAN_2:
      // FASE 2: BARRIDO AL LADO CONTRARIO (Por si nos equivocamos o dimos la vuelta)
      // Invertimos
      if (lastSeenSide == -1) {
         // Girar Derecha
         setMotor(velGiroRapido, velGiroLento); 
      } else {
         // Girar Izquierda
         setMotor(velGiroLento, velGiroRapido); 
      }
      
      if (now - recT0 > T_BARRIDO_2) { 
        recStage = REC_CENTER; 
        recT0 = now; 
      }
      break;

    case REC_CENTER:
       setMotor(0,0); // Rendirse
       break;
  }
}