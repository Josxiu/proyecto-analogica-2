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
//  CALIBRACIÓN
// =========================
int umbralL = 1200;
int umbralC = 1200;
int umbralR = 1500; 

// =========================
//  TUNING
// =========================
int velMax = 185;       

// RECUPERACIÓN (ARCO INVERTIDO)
int velGiroRapido = 180; // La rueda que empuja
int velGiroLento = -110;  // La rueda que hace de ancla (Negativo para agarre)

// TIEMPOS
const int TIEMPO_REVERSA = 200; 
const int TIEMPO_SCAN_1 = 700;  
const int TIEMPO_SCAN_2 = 1400; 

// PID
float Kp = 55.0; 
float Kd = 45.0; 
float factorFreno = 0.9; 
const int MIN_POWER = 120; 

// =========================
//  VARIABLES
// =========================
int lastError = 0;       
int lastSeenSide = 1; 

enum RecStage { REC_IDLE = 0, REC_BRAKE, REC_REVERSE, REC_SCAN_1, REC_SCAN_2, REC_CENTER };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;

// Amortiguador
int lastSpeedLeft = 0;
int lastSpeedRight = 0;
const int MAX_ACEL = 30; 

// =========================
//  FUNCIONES
// =========================
int suavizarCambio(int velocidadObjetivo, int velocidadAnterior) {
  int diferencia = velocidadObjetivo - velocidadAnterior;
  if (diferencia > MAX_ACEL) return velocidadAnterior + MAX_ACEL;
  else if (diferencia < -MAX_ACEL) return velocidadAnterior - MAX_ACEL;
  return velocidadObjetivo;
}

int corregirZonaMuerta(int pwm) {
  if (abs(pwm) > 0 && abs(pwm) < MIN_POWER) {
    return (pwm > 0) ? MIN_POWER : -MIN_POWER;
  }
  return pwm;
}

void setMotor(int speedLeft, int speedRight) {
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);
  
  speedLeft = suavizarCambio(speedLeft, lastSpeedLeft);
  speedRight = suavizarCambio(speedRight, lastSpeedRight);
  lastSpeedLeft = speedLeft;
  lastSpeedRight = speedRight;

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
  Serial.println("LOGICA INVERTIDA + ARCO LISTA");
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

    int velocidadBase = velMax;
    if (abs(error) == 3) velocidadBase = 110; 
    else velocidadBase = velMax - (abs(correccionPID) * factorFreno);
    
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
//  RECUPERACIÓN CORREGIDA (SENTIDO INVERTIDO)
// =========================
void manejarRecuperacion() {
  unsigned long now = millis();
  int velBack = -160;

  switch (recStage) {
    case REC_IDLE:
      recStage = REC_BRAKE;
      recT0 = now;
      setMotor(-255, -255); 
      break;

    case REC_BRAKE:
      if (now - recT0 > 100) { recStage = REC_REVERSE; recT0 = now; }
      break;

    case REC_REVERSE:
      setMotor(velBack, velBack);
      if (now - recT0 > TIEMPO_REVERSA) { recStage = REC_SCAN_1; recT0 = now; }
      break;

    case REC_SCAN_1:
      // AQUI ESTABA EL CAMBIO:
      // Si la última vez fue -1 (Izq), TÚ querías que girara a la DERECHA.
      
      if (lastSeenSide == -1) {
         // Memoria: IZQUIERDA -> Acción: Girar DERECHA
         // Para girar Derecha: Izq Rápido, Der Lento/Atrás
         setMotor(velGiroRapido, velGiroLento); 
      } else {
         // Memoria: DERECHA -> Acción: Girar IZQUIERDA
         // Para girar Izquierda: Izq Lento/Atrás, Der Rápido
         setMotor(velGiroLento, velGiroRapido); 
      }
      
      if (now - recT0 > TIEMPO_SCAN_1) { 
        recStage = REC_SCAN_2; 
        recT0 = now; 
      }
      break;

    case REC_SCAN_2:
      // FASE 2: Invertimos
      if (lastSeenSide == -1) {
         // Ahora giro IZQ
         setMotor(velGiroLento, velGiroRapido); 
      } else {
         // Ahora giro DER
         setMotor(velGiroRapido, velGiroLento);
      }
      
      if (now - recT0 > TIEMPO_SCAN_2) { 
        recStage = REC_CENTER; 
        recT0 = now; 
      }
      break;

    case REC_CENTER:
       setMotor(0,0); 
       break;
  }
}