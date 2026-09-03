#define LED_PIN 8

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("LED test starting...");
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED ON");
  delay(500);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED OFF");
  delay(500);
}