const int sensorIzq = 6;
const int sensorCen = 7;
const int sensorDer = 8;

void setup() {
  Serial.begin(115200);
  pinMode(sensorIzq, INPUT);
  pinMode(sensorCen, INPUT);
  pinMode(sensorDer, INPUT);
}

void loop() {
  Serial.print("Izq: "); Serial.print(digitalRead(sensorIzq));
  Serial.print(" | Cen: "); Serial.print(digitalRead(sensorCen));
  Serial.print(" | Der: "); Serial.println(digitalRead(sensorDer));
  delay(200);
}
