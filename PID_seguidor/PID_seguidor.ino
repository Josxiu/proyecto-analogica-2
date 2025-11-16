#include <Bluepad32.h>

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
ControllerPtr ctl;
bool modoAuto = false;            // false = manual, true = seguidor
uint8_t velocidadBase = 200;

// ===== Recovery / búsqueda -----
enum RecStage { REC_IDLE = 0, REC_BACKUP, REC_TURN_TO_LAST, REC_SPIN, REC_DONE };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;
int lastSeen = 0; // 0 = desconocido, 1 = centro, 2 = izquierda, 3 = derecha
int spinDir = 1;  // 1 = derecha, -1 = izquierda (alternar)
const unsigned long BACKUP_MS = 180;
const unsigned long TURN_MS = 700;
const unsigned long SPIN_MS = 1200;

// ===== Callbacks Bluepad32 =====
void onConnectedController(ControllerPtr c) {
  ctl = c;
  Serial.println("Control conectado.");
}
void onDisconnectedController(ControllerPtr c) {
  ctl = nullptr;
  Serial.println("Control desconectado.");
}

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
  // tu implementación de giro (funciona con tu cableado)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(canalA, pwm);
  ledcWrite(canalB, pwm);
}

void derecha(uint8_t pwm) {
  // tu implementación de giro (funciona con tu cableado)
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(canalA, pwm);
  ledcWrite(canalB, pwm);
}

// ===== Variables PID =====
float Kp = 1.0;  // Ganancia proporcional (ajusta)
float Ki = 0.0;  // Integral (opcional, empieza en 0)
float Kd = 0.5;  // Derivativo (ajusta)
float errorPrev = 0.0;
float integral = 0.0;

// ===== Lógica del seguidor con PID =====
void seguirLinea() {
  int rawL = digitalRead(S_LEFT);
  int rawC = digitalRead(S_CENTER);
  int rawR = digitalRead(S_RIGHT);

  // Invertir: true = negro (línea)
  bool L = !rawL;
  bool C = !rawC;
  bool R = !rawR;

  // Si vemos la línea, cancelar recovery y actualizar lastSeen
  if (L || C || R) {
    recStage = REC_IDLE;
    if (C) lastSeen = 1;
    else if (L) lastSeen = 2;
    else if (R) lastSeen = 3;
  }

  // Calcular error discreto basado en sensores
  float error = 0.0;
  if (C && !L && !R) error = 0.0;  // Centro
  else if (L && !C && !R) error = -1.0;  // Solo izquierda
  else if (R && !C && !L) error = 1.0;   // Solo derecha
  else if (C && L && !R) error = -0.5;   // Centro + izquierda
  else if (C && R && !L) error = 0.5;    // Centro + derecha
  else if (L && R && !C) error = 0.0;    // Ambos lados, sin centro (avanzar despacio)
  else if (L && C && R) error = 0.0;     // Todos (avanzar)

  // PID
  integral += error;
  float derivative = error - errorPrev;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  errorPrev = error;

  // Aplicar corrección a velocidades
  uint8_t pwmL = velocidadBase + correction * 50;  // Factor de ajuste (prueba 30-70)
  uint8_t pwmR = velocidadBase - correction * 50;
  pwmL = constrain(pwmL, 0, 255);
  pwmR = constrain(pwmR, 0, 255);

  // Si hay línea, usar PID; si no, recovery
  if (L || C || R) {
    adelante(pwmL, pwmR);
  } else {
    // --- RECOVERY ---
    unsigned long now = millis();
    if (recStage == REC_IDLE) {
      // iniciar recovery
      recStage = REC_BACKUP;
      recT0 = now;
      // alternar dirección de spin para la próxima vez si usamos SPIN
      spinDir = (spinDir > 0) ? -1 : 1;
      // si no hay lastSeen, preferimos spin en dirección spinDir
    }

    // Stage: retroceder un poco para salir de zona problemática
    if (recStage == REC_BACKUP) {
      if (now - recT0 < BACKUP_MS) {
        atras(120, 120);
        return;
      } else {
        recStage = REC_TURN_TO_LAST;
        recT0 = now;
      }
    }

    // Stage: girar hacia la última posición conocida
    if (recStage == REC_TURN_TO_LAST) {
      uint8_t turnP = max(110, velocidadBase * 6 / 10); // potencia de giro
      if (lastSeen == 2) {
        // girar a la izquierda buscando la línea
        if (now - recT0 < TURN_MS) {
          izquierda(turnP);
          return;
        } else {
          recStage = REC_SPIN;
          recT0 = now;
          return;
        }
      } else if (lastSeen == 3) {
        // girar a la derecha buscando la línea
        if (now - recT0 < TURN_MS) {
          derecha(turnP);
          return;
        } else {
          recStage = REC_SPIN;
          recT0 = now;
          return;
        }
      } else if (lastSeen == 1) {
        // si la última fue centro, intentar avanzar un poco y luego girar
        if (now - recT0 < (TURN_MS/3)) {
          adelante(velocidadBase/2, velocidadBase/2);
          return;
        } else {
          recStage = REC_SPIN;
          recT0 = now;
          return;
        }
      } else {
        // desconocido -> pasar a spin directamente
        recStage = REC_SPIN;
        recT0 = now;
        return;
      }
    }

    // Stage: spin amplio (girar en sitio) buscando la línea
    if (recStage == REC_SPIN) {
      uint8_t spinP = max(100, velocidadBase * 5 / 10);
      if (now - recT0 < SPIN_MS) {
        // girar en la dirección spinDir (1 = derecha, -1 = izquierda)
        if (spinDir > 0) derecha(spinP);
        else izquierda(spinP);
        return;
      } else {
        // si no encontró, cambiar a REC_DONE (detener) o volver a intentar ciclo
        recStage = REC_DONE;
        recT0 = now;
        return;
      }
    }

    // Stage final: detener y esperar (puedes cambiar a búsqueda continua si prefieres)
    if (recStage == REC_DONE) {
      detener();
      return;
    }
  }
}

// ===== Control manual =====
void controlManual() {
  if (!ctl) { detener(); return; }

  uint8_t dpad = ctl->dpad();
  bool up = dpad & 0x01;
  bool down = dpad & 0x02;
  bool left = dpad & 0x08;
  bool right = dpad & 0x04;

  int l2 = ctl->brake();
  int r2 = ctl->throttle();
  uint8_t pwmL2 = map(l2, 0, 1023, 0, 255);
  uint8_t pwmR2 = map(r2, 0, 1023, 0, 255);

  if (r2 > 30) adelante(pwmR2, pwmR2);
  else if (l2 > 30) atras(pwmL2, pwmL2);
  else if (left) izquierda(200);
  else if (right) derecha(200);
  else if (up) adelante(200, 200);
  else if (down) atras(200, 200);
  else detener();
}

// ===== Procesar controlador =====
void processController() {
  // gestionar toggle botón A si existe el control
  if (ctl) {
    static unsigned long lastToggle = 0;
    bool botonA = ctl->a();
    unsigned long now = millis();
    if (botonA && (now - lastToggle > 300)) {
      modoAuto = !modoAuto;
      Serial.printf("Modo: %s\n", modoAuto ? "SEGUIDOR" : "MANUAL");
      lastToggle = now;
      // reset recovery al cambiar modo
      recStage = REC_IDLE;
    }
  }

  // ejecutar modo
  if (modoAuto) {
    seguirLinea();
    return;
  }

  if (ctl) controlManual();
  else detener();
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  // Pines IN
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Pines EN (las ledc attach los necesitan)
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // PWM
  ledcSetup(canalA, freqPWM, resolucion);
  ledcSetup(canalB, freqPWM, resolucion);
  ledcAttachPin(ENA, canalA);
  ledcAttachPin(ENB, canalB);

  // Sensores
  pinMode(S_LEFT, INPUT);
  pinMode(S_CENTER, INPUT);
  pinMode(S_RIGHT, INPUT);

  detener();
  Serial.println("Listo. Presiona A para cambiar de modo.");
}

// ===== Loop =====
void loop() {
  BP32.update();
  processController();
  // loop rápido para buena reactividad
  delay(15);
}
