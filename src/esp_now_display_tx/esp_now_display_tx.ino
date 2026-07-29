// ESP-NOW Mock GPS Transmitter
// No display wiring needed on this device — just power/USB.
// Sends a fake, slowly-drifting GPS fix as text every 2 seconds.
//
// Update rxMac[] below with the RECEIVER's real MAC address once you know
// it (the RX sketch prints its own MAC to Serial on boot). 0xFF broadcast
// works for now without any pairing.

#include <WiFi.h>
#include <esp_now.h>

// Must match the receiver's struct exactly
typedef struct struct_message {
  char text[64];
} struct_message;

struct_message outgoingMsg;

// Placeholder broadcast address - swap in the receiver's real MAC later
uint8_t rxMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Mocked GPS state - random-walks around a starting point so the display
// shows something changing over time
double mockLat = 37.7749;
double mockLon = -122.4194;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

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

void loop() {
  // Small random drift, roughly ~10m per step at these latitudes
  mockLat += (random(-10, 11)) / 100000.0;
  mockLon += (random(-10, 11)) / 100000.0;
  int mockSats = 4 + random(0, 8);

  snprintf(outgoingMsg.text, sizeof(outgoingMsg.text),
           "%.5f,%.5f\nSATS:%d FIX:3D", mockLat, mockLon, mockSats);

  esp_err_t result = esp_now_send(rxMac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));

  Serial.print(result == ESP_OK ? "Sent: " : "Send error: ");
  Serial.println(outgoingMsg.text);

  delay(2000);
}
