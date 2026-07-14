#include <esp_now.h>
#include <WiFi.h>

// 24:EC:4A:CB:1E:0C
uint8_t receiverMac[] = {0x24, 0xEC, 0x4A, 0xCB, 0x1E, 0x0C};

typedef struct {
  bool ledOn;
} Message;

bool ledState = false;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Sender ready");
}

void loop() {
  ledState = !ledState;

  Message msg;
  msg.ledOn = ledState;
  esp_now_send(receiverMac, (uint8_t *)&msg, sizeof(msg));
  Serial.printf("Sent: ledOn = %d\n", ledState);

  delay(1000);
}