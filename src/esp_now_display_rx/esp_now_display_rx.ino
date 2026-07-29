// ESP-NOW Display Receiver
// Wiring: same as original display.ino — SH1106 OLED over I2C
//   SDA -> GPIO6, SCL -> GPIO7 (ESP32-C6 Pico)
//
// Prints incoming ESP-NOW text to Serial and shows it on the OLED.
// Update rxMac's *sender* counterpart (txMac in the TX sketch) once you
// know the real MAC addresses — this uses 0xFF broadcast as a placeholder
// so it'll pick up messages from any sender for now.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <esp_now.h>

// ESP32-C6 Pico I2C pins
#define I2C_SDA 6
#define I2C_SCL 7

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Must match the transmitter's struct exactly
typedef struct struct_message {
  char text[64];
} struct_message;

struct_message incomingMsg;
volatile bool newMessage = false;
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT_MS = 5000;

// arduino-esp32 core 3.x callback signature (esp_now_recv_info_t*, not raw mac)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(struct_message)) {
    Serial.printf("Received unexpected length: %d\n", len);
    return;
  }
  memcpy(&incomingMsg, data, sizeof(incomingMsg));
  newMessage = true;
  lastReceiveTime = millis();

  Serial.printf("From %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5],
                incomingMsg.text);
}

void showText(const char *msg) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(msg);
  display.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("Display not found!");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println("Waiting");
  display.println("for data");
  display.display();

  WiFi.mode(WIFI_STA);
  Serial.print("This device's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(put this address in the TX sketch's rxMac[] once known)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (newMessage) {
    newMessage = false;
    showText(incomingMsg.text);
  }

  if (lastReceiveTime != 0 && millis() - lastReceiveTime > TIMEOUT_MS) {
    showText("No data\n(timeout)");
    lastReceiveTime = 0;
  }

  delay(50);
}

