/*
 * DRONE - ESP32-C3 GPS UART Read + TinyGPS++ Parse -> ESP-NOW TX
 * -----------------------------------------------------------------
 * Two jobs, both over ESP-NOW to the controller:
 *
 *   1. Reports its own GPS fix (~1x/sec) so the controller can run
 *      the FOLLOW HOME distance math (drone <-> captured launch point).
 *   2. Is the actual "seeker" for puck-finding: listens for the
 *      puck's RSSI beacon (beacon_t, ~10x/sec), filters + classifies
 *      it into a NEAR/MID/FAR/LOST zone (same logic as Brayden's
 *      esp_now_proximity_finder.ino SEEKER role), and relays that
 *      zone + filtered RSSI to the controller (~10x/sec) as a
 *      proximity_report_t. The controller just plays the corresponding
 *      beep pattern - it doesn't do any RSSI math itself. This is the
 *      drone<->puck link players actually use to find the puck; GPS
 *      is not part of that path anymore.
 *
 * PASSIVE LISTENER (GPS UART): TX pin is passed as -1, never drives
 * the tapped wire.
 *
 * Wiring:
 *   GPS TXD (tapped wire) -> ESP32 GPIO20 (RX only)
 *   GND                   -> ESP32 GND
 *
 * Library needed: TinyGPS++ (search "TinyGPSPlus" by Mikal Hart
 * in the Arduino Library Manager - NOT the old "TinyGPS" library,
 * which has a different API).
 *
 * Update rxMac[] below with the controller's real MAC address once
 * you know it (controller.ino prints its own MAC to Serial on boot).
 * 0xFF broadcast works for now without any pairing - note this means
 * this sketch will also pick up broadcast beacons from ANY nearby
 * puck, not just yours, until you swap in a real peer MAC.
 *
 * WIRE PROTOCOL - all three must match field-for-field across sketches:
 *   struct_message     - controller.ino, puck_gps_tx.ino (unused there
 *                         now, but the type is still shared)
 *   beacon_t            - puck_gps_tx.ino (sender), classified here
 *   proximity_report_t   - controller.ino (receiver)
 *
 * Fixed WiFi channel below must match controller.ino and
 * puck_gps_tx.ino so all three radios are reliably on the same
 * channel and RSSI stays comparable.
 */

#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define GPS_RX_PIN 20
#define GPS_BAUD   115200

#define WIFI_CHANNEL 1
const uint8_t TX_POWER_QUARTER_DBM = 84; // 84 = 21 dBm (max) - keep identical on all radios

const uint32_t GPS_SEND_INTERVAL_MS       = 1000; // struct_message, for FOLLOW HOME
const uint32_t PROXIMITY_SEND_INTERVAL_MS = 100;   // proximity_report_t, to controller

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// ---- GPS struct (drone's own fix, for FOLLOW HOME) ----
// Must match controller.ino / puck_gps_tx.ino exactly
enum DeviceId : uint8_t { DEVICE_DRONE = 0, DEVICE_PUCK = 1 };

typedef struct struct_message {
  uint8_t deviceId;
  bool fixValid;
  double lat;
  double lon;
  uint8_t sats;
} struct_message;

struct_message outgoingGpsMsg = { DEVICE_DRONE, false, 0, 0, 0 };

// ---- RSSI beacon from the puck ----
// Must match puck_gps_tx.ino / controller.ino exactly
typedef struct {
  uint32_t counter;
} beacon_t;

// ---- Zone classification (same thresholds as esp_now_proximity_finder.ino) ----
// Recalibrate together if these change, or the drone and the standalone
// bench-test sketch will disagree about what counts as "near."
enum PuckZone { ZONE_NEAR = 0, ZONE_MID = 1, ZONE_FAR = 2, ZONE_LOST = 3 };
const int RSSI_NEAR = -55;
const int RSSI_MID  = -70;
const int RSSI_FAR  = -82;
const int RSSI_HYSTERESIS = 4;
const float EMA_ALPHA = 0.15f;
const unsigned long RSSI_SIGNAL_LOST_MS = 800;

float emaRssi = 0;
bool haveFirstRssiReading = false;
unsigned long lastBeaconMillis = 0;
int puckZone = ZONE_LOST;

int classifyZone(float rssi, int prevZone) {
  int nearThresh = RSSI_NEAR, midThresh = RSSI_MID, farThresh = RSSI_FAR;
  // widen the threshold needed to LEAVE the current zone (hysteresis)
  if (prevZone == ZONE_NEAR) nearThresh -= RSSI_HYSTERESIS;
  if (prevZone == ZONE_MID)  midThresh  -= RSSI_HYSTERESIS;
  if (prevZone == ZONE_FAR)  farThresh  -= RSSI_HYSTERESIS;

  if (rssi >= nearThresh) return ZONE_NEAR;
  if (rssi >= midThresh)  return ZONE_MID;
  if (rssi >= farThresh)  return ZONE_FAR;
  return ZONE_LOST;
}

void updatePuckZone() {
  unsigned long now = millis();
  if (haveFirstRssiReading && (now - lastBeaconMillis <= RSSI_SIGNAL_LOST_MS)) {
    puckZone = classifyZone(emaRssi, puckZone);
  } else {
    puckZone = ZONE_LOST;
  }
}

// ---- Relay to controller: this drone's read on the puck's proximity ----
// Must match controller.ino exactly
typedef struct {
  float rssi;
  uint8_t zone;
} proximity_report_t;

// Placeholder broadcast address - swap in the controller's real MAC later
uint8_t rxMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Quiet on purpose - fires ~11x/sec (1 GPS + 10 proximity) and would
  // otherwise flood Serial. Uncomment for debugging sends:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}

// Incoming beacons from the puck
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(beacon_t)) return; // not a beacon - ignore

  int rssi = info->rx_ctrl->rssi;
  lastBeaconMillis = millis();
  if (!haveFirstRssiReading) {
    emaRssi = rssi;
    haveFirstRssiReading = true;
  } else {
    emaRssi = EMA_ALPHA * rssi + (1.0f - EMA_ALPHA) * emaRssi;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting GPS parse + ESP-NOW TX/RX (drone)...");

  // -1 for TX pin = RX only, never drives the tapped wire
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);

  Serial.print("This device's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(put this address in the controller's peer list once known)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

unsigned long lastGpsSend = 0;
unsigned long lastProximitySend = 0;

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  updatePuckZone();

  unsigned long now = millis();

  if (now - lastProximitySend >= PROXIMITY_SEND_INTERVAL_MS) {
    lastProximitySend = now;
    proximity_report_t report = { emaRssi, (uint8_t)puckZone };
    esp_now_send(rxMac, (uint8_t *)&report, sizeof(report));
  }

  if (now - lastGpsSend >= GPS_SEND_INTERVAL_MS) {
    lastGpsSend = now;

    outgoingGpsMsg.fixValid = gps.location.isValid();
    outgoingGpsMsg.lat = outgoingGpsMsg.fixValid ? gps.location.lat() : 0.0;
    outgoingGpsMsg.lon = outgoingGpsMsg.fixValid ? gps.location.lng() : 0.0;
    outgoingGpsMsg.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;

    esp_err_t result = esp_now_send(rxMac, (uint8_t *)&outgoingGpsMsg, sizeof(outgoingGpsMsg));

    Serial.printf("%s fix=%d lat=%.6f lon=%.6f sats=%d  zone=%d rssi=%.1f\n",
      result == ESP_OK ? "Sent:" : "Send error:",
      outgoingGpsMsg.fixValid, outgoingGpsMsg.lat, outgoingGpsMsg.lon, outgoingGpsMsg.sats,
      puckZone, emaRssi);

    // Same sanity check as the original serial-only version - if this
    // stays at 0 while you're clearly seeing NMEA text elsewhere, it
    // usually means TinyGPS++ isn't getting bytes fast enough.
    if (gps.charsProcessed() < 10 && millis() > 5000) {
      Serial.println("WARNING: No NMEA data seen yet - check wiring.");
    }
  }
}
