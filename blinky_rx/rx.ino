#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 8

typedef struct {
  bool ledOn;
} Message;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Message msg;
  memcpy(&msg, data, sizeof(msg));
  digitalWrite(LED_PIN, msg.ledOn ? HIGH : LOW);
  Serial.printf("Received: ledOn = %d\n", msg.ledOn);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Receiver ready");
}

void loop() {}