// ===== Pines motores =====
const int ENA = 2;
const int IN1 = 42;
const int IN2 = 41;
const int ENB = 36;
const int IN3 = 38;
const int IN4 = 37;

// ===== Pines sensores =====
const int S_LEFT = 6;
const int S_CENTER = 7;
const int S_RIGHT = 8;

// ===== Configuración PWM =====
const int canalA = 0;
const int canalB = 1;
const int freqPWM = 5000;
const int resolucion = 8;

// ===== Variables globales =====
uint8_t velocidadBase = 200;

// ===== Recovery / búsqueda -----
enum RecStage { REC_IDLE = 0, REC_BACKUP, REC_TURN_TO_LAST, REC_SPIN, REC_SPIRAL, REC_DONE };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;
int lastSeen = 0;       // 1 = centro, 2 = izquierda, 3 = derecha
int spinDir = 1;        // dirección alterna
int recoveryAttempts = 0;

const unsigned long BACKUP_MS = 150;
const unsigned long TURN_MS   = 500;
const unsigned long SPIN_MS   = 1000;
const unsigned long SPIRAL_MS = 800;

// ===== Funciones de movimiento =====
void detener() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(canalA, 0);
  ledcWrite(canalB, 0);
}

void adelante(uint8_t pwmA, uint8_t pwmB) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(canalA, pwmA);
  ledcWrite(canalB, pwmB);
}

void atras(uint8_t pwmA, uint8_t pwmB) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(canalA, pwmA);
  ledcWrite(canalB, pwmB);
}

void izquierda(uint8_t pwm) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(canalA, pwm);
  ledcWrite(canalB, pwm);
}

void derecha(uint8_t pwm) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(canalA, pwm);
  ledcWrite(canalB, pwm);
}

// ===== Lógica seguidor =====
void seguirLinea() {
  int rawL = digitalRead(S_LEFT);
  int rawC = digitalRead(S_CENTER);
  int rawR = digitalRead(S_RIGHT);

  bool L = !rawL;
  bool C = !rawC;
  bool R = !rawR;

  if (L || C || R) {
    recStage = REC_IDLE;
    if (C) lastSeen = 1;
    else if (L) lastSeen = 2;
    else if (R) lastSeen = 3;
  }

  // --- Casos normales ---
  if (C && !L && !R) { adelante(velocidadBase, velocidadBase); return; }
  if (C && L && !R)   { adelante(velocidadBase/2, velocidadBase); return; }
  if (C && R && !L)   { adelante(velocidadBase, velocidadBase/2); return; }
  if (L && !C && !R)  { izquierda(velocidadBase - 30); return; }
  if (R && !C && !L)  { derecha(velocidadBase - 30); return; }
  if (L && R && !C)   { adelante(velocidadBase/2, velocidadBase/2); return; }
  if (L && C && R)    { adelante(velocidadBase, velocidadBase); return; }

  // ===== Recovery (cuando NO detecta línea) =====
  unsigned long now = millis();

  if (recStage == REC_IDLE) {
    recStage = REC_BACKUP;
    recT0 = now;
    recoveryAttempts = 0;
    spinDir = (spinDir > 0) ? -1 : 1;
  }

  if (recStage == REC_BACKUP) {
    if (now - recT0 < BACKUP_MS) { atras(140, 140); return; }
    recStage = REC_TURN_TO_LAST; recT0 = now;
  }

  if (recStage == REC_TURN_TO_LAST) {
    uint8_t turnP = max(120, velocidadBase * 7/10);
    if (lastSeen == 2 && now - recT0 < TURN_MS) { izquierda(turnP); return; }
    if (lastSeen == 3 && now - recT0 < TURN_MS) { derecha(turnP); return; }
    recStage = REC_SPIN; recT0 = now;
  }

  if (recStage == REC_SPIN) {
    uint8_t spinP = max(120, velocidadBase * 6/10);
    if (now - recT0 < SPIN_MS) {
      if (spinDir > 0) derecha(spinP);
      else izquierda(spinP);
      return;
    }
    recoveryAttempts++;
    recStage = (recoveryAttempts < 3) ? REC_SPIRAL : REC_DONE;
    recT0 = now;
  }

  if (recStage == REC_SPIRAL) {
    uint8_t sp = max(100, velocidadBase * 5/10);
    if (now - recT0 < SPIRAL_MS) {
      if (spinDir > 0) adelante(sp/2, sp);
      else adelante(sp, sp/2);
      return;
    }
    recoveryAttempts++;
    recStage = (recoveryAttempts < 3) ? REC_SPIN : REC_DONE;
    spinDir = -spinDir;
    recT0 = now;
  }

  if (recStage == REC_DONE) {
    detener();
    return;
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ledcSetup(canalA, freqPWM, resolucion);
  ledcSetup(canalB, freqPWM, resolucion);
  ledcAttachPin(ENA, canalA);
  ledcAttachPin(ENB, canalB);

  pinMode(S_LEFT, INPUT);
  pinMode(S_CENTER, INPUT);
  pinMode(S_RIGHT, INPUT);

  detener();
  Serial.println("Iniciando en modo SEGUIDOR DE LÍNEA");
}

// ===== Loop =====
void loop() {
  seguirLinea();
  delay(15);
}
