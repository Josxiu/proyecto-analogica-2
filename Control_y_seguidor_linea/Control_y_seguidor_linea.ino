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
enum RecStage { REC_IDLE = 0, REC_BACKUP, REC_TURN_TO_LAST, REC_SPIN, REC_SPIRAL, REC_DONE };
RecStage recStage = REC_IDLE;
unsigned long recT0 = 0;
int lastSeen = 0; // 0 = desconocido, 1 = centro, 2 = izquierda, 3 = derecha
int spinDir = 1;  // 1 = derecha, -1 = izquierda (alternar)
int recoveryAttempts = 0;  // Contador de intentos
const unsigned long BACKUP_MS = 150;  // Reducido
const unsigned long TURN_MS = 500;    // Reducido
const unsigned long SPIN_MS = 1000;   // Reducido
const unsigned long SPIRAL_MS = 800; // Nuevo para espiral

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

// ===== Lógica del seguidor (robusta) =====
void seguirLinea() {
  int rawL = digitalRead(S_LEFT);
  int rawC = digitalRead(S_CENTER);
  int rawR = digitalRead(S_RIGHT);

  // Sensores: 0 = negro, 1 = blanco -> invertimos para true = negro
  bool L = !rawL;
  bool C = !rawC;
  bool R = !rawR;

  // Si vemos la línea en cualquier sensor, cancelamos recovery
  if (L || C || R) {
    recStage = REC_IDLE;
    // actualizar lastSeen (prioridad centro, luego izquierda, luego derecha)
    if (C) lastSeen = 1;
    else if (L) lastSeen = 2;
    else if (R) lastSeen = 3;
  }

  // --- CASOS posibles con prioridad y correcciones suaves ---
  // 1) Centro solo -> adelante
  if (C && !L && !R) {
    adelante(velocidadBase, velocidadBase);
    return;
  }

  // 2) Centro + izquierda -> corrección suave a la izquierda
  if (C && L && !R) {
    // reduce izquierda (hacer que el motor izquierdo vaya más lento)
    // dependiendo de tu wiring esto puede ser adelantar con uno más lento
    adelante(velocidadBase/2, velocidadBase);
    return;
  }

  // 3) Centro + derecha -> corrección suave a la derecha
  if (C && R && !L) {
    adelante(velocidadBase, velocidadBase/2);
    return;
  }

  // 4) Izquierda solo -> giro fuerte/ajuste a la izquierda
  if (L && !C && !R) {
    izquierda(velocidadBase - 30);
    return;
  }

  // 5) Derecha solo -> giro fuerte/ajuste a la derecha
  if (R && !C && !L) {
    derecha(velocidadBase - 30);
    return;
  }

  // 6) Izquierda + derecha, centro apagado (línea ancha / cruce) -> avanzar despacio
  if (L && R && !C) {
    adelante(velocidadBase/2, velocidadBase/2);
    return;
  }

  // 7) Todos detectan (cruce o zona negra) -> adelante
  if (L && C && R) {
    adelante(velocidadBase, velocidadBase);
    return;
  }

  // 8) Ninguno detecta -> recuperación (state machine, no bloqueante)
  unsigned long now = millis();
  if (recStage == REC_IDLE) {
    // iniciar recovery
    recStage = REC_BACKUP;
    recT0 = now;
    recoveryAttempts = 0;  // Reset contador
    spinDir = (spinDir > 0) ? -1 : 1;  // Alternar dirección
  }

  // Stage: retroceder un poco para salir de zona problemática
  if (recStage == REC_BACKUP) {
    if (now - recT0 < BACKUP_MS) {
      atras(140, 140);  // Aumenta velocidad
      return;
    } else {
      recStage = REC_TURN_TO_LAST;
      recT0 = now;
    }
  }

  // Stage: girar hacia la última posición conocida
  if (recStage == REC_TURN_TO_LAST) {
    uint8_t turnP = max(120, velocidadBase * 7 / 10);  // Aumenta potencia
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
    } else {
      // Desconocido o centro: ir a SPIN directamente
      recStage = REC_SPIN;
      recT0 = now;
      return;
    }
  }

  // Stage: spin amplio (girar en sitio) buscando la línea
  if (recStage == REC_SPIN) {
    uint8_t spinP = max(120, velocidadBase * 6 / 10);  // Aumenta potencia
    if (now - recT0 < SPIN_MS) {
      // girar en la dirección spinDir (1 = derecha, -1 = izquierda)
      if (spinDir > 0) derecha(spinP);
      else izquierda(spinP);
      return;
    } else {
      recoveryAttempts++;
      if (recoveryAttempts < 3) {
        recStage = REC_SPIRAL;
        recT0 = now;
        return;
      } else {
        recStage = REC_DONE;
        recT0 = now;
        return;
      }
    }
  }

  // Nueva Stage: búsqueda en espiral (giro con avance lento para cubrir área)
  if (recStage == REC_SPIRAL) {
    uint8_t spiralP = max(100, velocidadBase * 5 / 10);
    if (now - recT0 < SPIRAL_MS) {
      // Giro en espiral: combina giro con avance
      if (spinDir > 0) {
        adelante(spiralP / 2, spiralP);  // Avanza más a la derecha
      } else {
        adelante(spiralP, spiralP / 2);  // Avanza más a la izquierda
      }
      return;
    } else {
      recoveryAttempts++;
      if (recoveryAttempts < 3) {
        recStage = REC_SPIN;  // Vuelve a SPIN con dirección alterna
        spinDir = -spinDir;
        recT0 = now;
        return;
      } else {
        recStage = REC_DONE;
        recT0 = now;
        return;
      }
    }
  }

  // Stage final: detener y esperar
  if (recStage == REC_DONE) {
    detener();
    return;
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
