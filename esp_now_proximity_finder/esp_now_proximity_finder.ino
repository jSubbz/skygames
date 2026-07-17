/*
  ============================================================================
  ESP-NOW PROXIMITY FINDER  —  Waveshare ESP32-S3 Zero x2
  ============================================================================

  One board = "HIDER"  -> broadcasts a beacon packet ~10x/sec.
  One board = "SEEKER" -> listens for beacons, reads RSSI, filters it,
                           buckets it into NEAR / MID / FAR / LOST, and
                           drives a passive piezo buzzer with a distinct
                           beep pattern per zone (fast+high when close,
                           slow+low when far).

  HOW TO FLASH:
    1. Open this file in Arduino IDE.
    2. Board: "ESP32S3 Dev Module" (install "esp32 by Espressif Systems"
       boards package, version 3.x, if you haven't already).
       - USB CDC On Boot: "Enabled" (needed for Serial over the native
         USB-C port on the S3 Zero)
    3. Set DEVICE_ROLE below to ROLE_HIDER, upload to board #1.
    4. Set DEVICE_ROLE below to ROLE_SEEKER, upload to board #2.
    5. Wire a passive piezo buzzer: + to BUZZER_PIN, - to GND.
    6. Open Serial Monitor at 115200 baud on the SEEKER to see raw/filtered
       RSSI while you calibrate (see CALIBRATION section below).

  ============================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================================
// 1. SET THE ROLE FOR THIS BOARD BEFORE FLASHING
// ============================================================================
#define ROLE_HIDER  1
#define ROLE_SEEKER 2

#define DEVICE_ROLE ROLE_HIDER     // <-- change to ROLE_SEEKER for the 2nd board

// ============================================================================
// 2. RADIO CONFIG (must match on both boards)
// ============================================================================
#define WIFI_CHANNEL 1                     // any fixed channel 1-13
const uint8_t TX_POWER_QUARTER_DBM = 84;   // 84 = 21 dBm (max). Keep identical
                                            // on both boards so RSSI is comparable.
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t BEACON_INTERVAL_MS = 100;   // hider sends 10 beacons/sec
const uint32_t SIGNAL_LOST_MS     = 800;   // seeker declares "lost" if no
                                            // beacon received in this long

typedef struct {
  uint32_t counter;
} beacon_t;

// ============================================================================
// 3. CALIBRATION — TUNE THESE FOR YOUR ENVIRONMENT
// ============================================================================
// Walk the hider to 10 ft, 25 ft, and 50 ft from the seeker in your real
// deployment environment (same enclosure, same antenna orientation you'll
// actually use). Watch the "raw" RSSI printed to Serial at each distance,
// average it, and set the thresholds below to fall between your measured
// values. These defaults are a reasonable indoor starting point but WILL
// need adjustment for your specific hardware/enclosure/environment.
const int RSSI_NEAR = -55;   // stronger than this => NEAR (~10 ft)
const int RSSI_MID  = -70;   // stronger than this => MID  (~25 ft)
const int RSSI_FAR  = -82;   // stronger than this => FAR  (~50 ft)
                              // weaker than RSSI_FAR, or no packets => LOST

const int RSSI_HYSTERESIS = 4;   // dB dead-zone to stop flicker at boundaries
const float EMA_ALPHA = 0.15f;   // 0-1, lower = smoother but slower to react

// ============================================================================
// 4. SEEKER OUTPUT (buzzer)
// ============================================================================
const int BUZZER_PIN = 5;   // passive piezo: + here, - to GND

enum Zone { ZONE_NEAR = 0, ZONE_MID = 1, ZONE_FAR = 2, ZONE_LOST = 3 };
const char* zoneNames[] = {"NEAR", "MID", "FAR", "LOST"};

struct BeepPattern {
  uint16_t freq;      // tone frequency, Hz (0 = silent)
  uint16_t onMs;       // how long the tone sounds within each cycle
  uint16_t periodMs;   // full cycle length (on + off)
};

// index matches Zone enum
BeepPattern patterns[4] = {
  {2400, 110, 150},    // NEAR — rapid, high pitched, almost continuous
  {1500, 120, 400},    // MID  — moderate beeping
  {900,  150, 1000},   // FAR  — slow, low-pitched beeps
  {0,    0,   0}        // LOST — silent
};

// ============================================================================
// STATE (seeker)
// ============================================================================
float emaRssi = 0;
bool haveFirstReading = false;
volatile uint32_t lastBeaconMillis = 0;
int currentZone = ZONE_LOST;

// ============================================================================
// STATE (hider)
// ============================================================================
uint32_t beaconCounter = 0;
uint32_t lastSendMillis = 0;

// ============================================================================
// COMMON SETUP
// ============================================================================
void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed! Halting.");
    while (true) delay(1000);
  }
}

// ============================================================================
#if DEVICE_ROLE == ROLE_HIDER
// ============================================================================

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // optional: no-op. Could blink an LED here on send failure.
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP-NOW Proximity Finder: HIDER ===");

  initEspNow();
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer!");
  }

  Serial.println("Broadcasting beacons...");
}

void loop() {
  uint32_t now = millis();
  if (now - lastSendMillis >= BEACON_INTERVAL_MS) {
    lastSendMillis = now;
    beacon_t beacon = { beaconCounter++ };
    esp_now_send(broadcastAddress, (uint8_t*)&beacon, sizeof(beacon));
  }
}

// ============================================================================
#elif DEVICE_ROLE == ROLE_SEEKER
// ============================================================================

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(beacon_t)) return;

  int rssi = info->rx_ctrl->rssi;
  lastBeaconMillis = millis();

  if (!haveFirstReading) {
    emaRssi = rssi;
    haveFirstReading = true;
  } else {
    emaRssi = EMA_ALPHA * rssi + (1.0f - EMA_ALPHA) * emaRssi;
  }
}

int classifyZone(float rssi, int prevZone) {
  int nearThresh = RSSI_NEAR;
  int midThresh  = RSSI_MID;
  int farThresh  = RSSI_FAR;

  // widen the threshold needed to LEAVE the current zone (hysteresis)
  if (prevZone == ZONE_NEAR) nearThresh -= RSSI_HYSTERESIS;
  if (prevZone == ZONE_MID)  midThresh  -= RSSI_HYSTERESIS;
  if (prevZone == ZONE_FAR)  farThresh  -= RSSI_HYSTERESIS;

  if (rssi >= nearThresh) return ZONE_NEAR;
  if (rssi >= midThresh)  return ZONE_MID;
  if (rssi >= farThresh)  return ZONE_FAR;
  return ZONE_LOST;
}

void updateBuzzer() {
  BeepPattern p = patterns[currentZone];
  if (p.periodMs == 0) {
    noTone(BUZZER_PIN);
    return;
  }
  uint32_t phase = millis() % p.periodMs;
  if (phase < p.onMs) {
    tone(BUZZER_PIN, p.freq);
  } else {
    noTone(BUZZER_PIN);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP-NOW Proximity Finder: SEEKER ===");

  pinMode(BUZZER_PIN, OUTPUT);
  initEspNow();
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Listening for hider beacons...");
  Serial.println("raw_rssi, filtered_rssi, zone");
}

void loop() {
  uint32_t now = millis();

  // signal-lost check
  if (haveFirstReading && (now - lastBeaconMillis > SIGNAL_LOST_MS)) {
    currentZone = ZONE_LOST;
  } else if (haveFirstReading) {
    currentZone = classifyZone(emaRssi, currentZone);
  }

  updateBuzzer();

  // debug print ~4x/sec for calibration
  static uint32_t lastPrint = 0;
  if (now - lastPrint > 250) {
    lastPrint = now;
    if (haveFirstReading) {
      Serial.printf("filtered_rssi=%.1f  zone=%s\n", emaRssi, zoneNames[currentZone]);
    } else {
      Serial.println("waiting for beacons...");
    }
  }
}

// ============================================================================
#else
#error "Set DEVICE_ROLE to ROLE_HIDER or ROLE_SEEKER above"
#endif
