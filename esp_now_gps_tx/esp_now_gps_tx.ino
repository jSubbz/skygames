/*
 * ESP32-C3 GPS UART Read + TinyGPS++ Parse -> ESP-NOW TX
 * --------------------------------------------------------
 * Same passive listener as gps_parse_test.ino, but instead of
 * printing decoded values to Serial, sends them over ESP-NOW as
 * text using the same struct_message pattern as
 * esp_now_display_rx.ino / esp_now_display_tx.ino.
 *
 * PASSIVE LISTENER: TX pin is passed as -1, never drives the
 * tapped wire.
 *
 * Wiring:
 *   GPS TXD (tapped wire) -> ESP32 GPIO20 (RX only)
 *   GND                   -> ESP32 GND
 *
 * Library needed: TinyGPS++ (search "TinyGPSPlus" by Mikal Hart
 * in the Arduino Library Manager - NOT the old "TinyGPS" library,
 * which has a different API).
 *
 * Update rxMac[] below with the display receiver's real MAC
 * address once you know it (esp_now_display_rx.ino prints its own
 * MAC to Serial on boot). 0xFF broadcast works for now without
 * any pairing.
 *
 * NOTE: struct_message here uses a 96-byte text buffer, larger
 * than the 64-byte one in the mock GPS example, since a full
 * fix/lat/lon/sats/alt/speed line runs longer than mock data did.
 * The RECEIVER's struct_message must match this size exactly, or
 * esp_now_display_rx.ino's length check will reject every packet.
 * If you're still also running esp_now_display_tx.ino (the mock),
 * bump its buffer to 96 too so both senders stay compatible with
 * the same receiver.
 */

#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <esp_now.h>

#define GPS_RX_PIN 20
#define GPS_BAUD   115200

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Must match the receiver's struct exactly (96 bytes here, see note above)
typedef struct struct_message {
  char text[96];
} struct_message;

struct_message outgoingMsg;

// Placeholder broadcast address - swap in the receiver's real MAC later
uint8_t rxMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting GPS parse + ESP-NOW TX...");

  // -1 for TX pin = RX only, never drives the tapped wire
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);

  WiFi.mode(WIFI_STA);
  Serial.print("This device's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(put this address in the RX sketch's txMac[] once known)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

unsigned long lastSend = 0;

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    snprintf(outgoingMsg.text, sizeof(outgoingMsg.text),
      "fix=%d lat=%.6f lon=%.6f\nsats=%d alt=%.1fm spd=%.1fm/s",
      gps.location.isValid(),
      gps.location.isValid() ? gps.location.lat() : 0.0,
      gps.location.isValid() ? gps.location.lng() : 0.0,
      gps.satellites.isValid() ? gps.satellites.value() : 0,
      gps.altitude.isValid() ? gps.altitude.meters() : 0.0f,
      gps.speed.isValid() ? gps.speed.mps() : 0.0f
    );

    esp_err_t result = esp_now_send(rxMac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));

    Serial.print(result == ESP_OK ? "Sent: " : "Send error: ");
    Serial.println(outgoingMsg.text);

    // Same sanity check as the original serial-only version - if this
    // stays at 0 while you're clearly seeing NMEA text elsewhere, it
    // usually means TinyGPS++ isn't getting bytes fast enough.
    if (gps.charsProcessed() < 10 && millis() > 5000) {
      Serial.println("WARNING: No NMEA data seen yet - check wiring.");
    }
  }
}
