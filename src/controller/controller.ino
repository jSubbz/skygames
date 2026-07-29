/*
 * Controller / Receiver - consolidated draft (v3)
 * -------------------------------------------------
 * Replaces the separate blinky_rx / esp_now_display_rx / esp_now_gps_rx
 * drafts with one sketch.
 *
 * Architecture (as of this draft):
 *   Puck  --beacon_t, ~10x/sec-->  Drone
 *   Drone --filters RSSI, classifies NEAR/MID/FAR/LOST zone (same
 *           logic as esp_now_proximity_finder.ino's SEEKER role) -->
 *   Drone --proximity_report_t, ~10x/sec-->  Controller (this board)
 *   Controller drives tone/LED off the zone the drone relays. No RSSI
 *   math happens here - the drone is the one flying around the puck,
 *   so it has to be the one sensing proximity; the controller just
 *   plays the result back to the operator.
 *
 *   Drone --struct_message (GPS), ~1x/sec--> Controller, for the
 *   separate FOLLOW HOME feature (drone <-> its own first fix, no
 *   controller GPS). Puck no longer sends GPS - drone<->puck finding
 *   is RSSI-only.
 *
 * Controls:
 *   - FOLLOW PUCK button: track the puck via the drone's relayed
 *     RSSI zone.
 *   - FOLLOW HOME button: track the way back to the captured home
 *     point.
 *   - Rotary encoder: dev/test override. Turning it feeds a manual
 *     distance into the tone/LED instead of live sensing; reverts to
 *     live data automatically 5s after you stop turning it.
 *
 * Screen content is still open (per team discussion) - updateDisplay()
 * below is a plain status readout, not a locked-in UI design. Treat it
 * as a placeholder to swap out once the "SET PUCK" screen concept
 * (ui_test.ino) is settled.
 *
 * WIRE PROTOCOL - three message types travel over ESP-NOW to this board:
 * ------------------------------------------------------------------
 *   struct_message (GPS) - sent ~1x/sec by esp_now_gps_tx.ino (drone
 *   only now). Must match field-for-field:
 *
 *     typedef struct struct_message {
 *       uint8_t deviceId;  // DEVICE_DRONE (0) or DEVICE_PUCK (1)
 *       bool fixValid;
 *       double lat;
 *       double lon;
 *       uint8_t sats;
 *     } struct_message;
 *
 *   proximity_report_t (RSSI relay) - sent ~10x/sec by the drone
 *   (esp_now_gps_tx.ino), which does its own RSSI filtering/zone
 *   classification against the puck's beacon. Must match exactly:
 *
 *     typedef struct { float rssi; uint8_t zone; } proximity_report_t;
 *     // zone: 0=NEAR, 1=MID, 2=FAR, 3=LOST
 *
 * The puck's beacon_t (see puck_gps_tx.ino / esp_now_proximity_finder.ino)
 * is NOT received here directly anymore - the drone is the one that
 * listens for it.
 *
 * All radios (controller, drone, puck) should lock to the same fixed
 * WiFi channel (see WIFI_CHANNEL below) for reliable reception.
 *
 * PIN CONFIG - CONFIRM AGAINST ACTUAL WIRING BEFORE FLASHING
 * ------------------------------------------------------------
 * I2C, encoder, and button pins below are copied from ui_test.ino,
 * since that's already been bench-tested against the real board.
 * LED_PIN and PIEZO_PIN are still unconfirmed placeholders.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ---- Pin configuration ----
#define I2C_SDA 6                    // matches ui_test.ino / esp_now_gps_rx.ino
#define I2C_SCL 7                    // matches ui_test.ino / esp_now_gps_rx.ino
#define LED_PIN 8                    // TODO confirm: tone-mirror LED (not yet bench-tested)
#define PIEZO_PIN 10                 // TODO confirm: piezo speaker (not yet bench-tested)
#define BUTTON_FOLLOW_PUCK_PIN 4     // matches ui_test.ino's BTN_CONFIRM
#define BUTTON_FOLLOW_HOME_PIN 5     // matches ui_test.ino's BTN_CANCEL
#define ENC_A 2                      // matches ui_test.ino's rotary encoder
#define ENC_B 3                      // matches ui_test.ino's rotary encoder
#define ENC_SW 1                     // matches ui_test.ino - optional, unused for now

// ---- OLED config (same as esp_now_gps_rx.ino / ui_test.ino) ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---- Radio config - must match esp_now_gps_tx.ino / puck_gps_tx.ino ----
#define WIFI_CHANNEL 1
const uint8_t TX_POWER_QUARTER_DBM = 84; // 84 = 21 dBm (max) - keep identical on all radios

// ---- FOLLOW HOME distance -> tone mapping. Tune once real-world distances are known ----
const float DIST_MIN_M = 1.0f;     // at or below this: max frequency ("hot")
const float DIST_MAX_M = 100.0f;   // at or above this: min frequency ("cold")
const int FREQ_MIN_HZ = 200;
const int FREQ_MAX_HZ = 1200;

// ---- Manual override (rotary encoder) config ----
const float OVERRIDE_DIST_MIN_M = 0.0f;
const float OVERRIDE_DIST_MAX_M = DIST_MAX_M;
const float ENCODER_STEP_M = 0.5f;                   // meters per detent, matches ui_test.ino
const unsigned long DIAL_OVERRIDE_TIMEOUT_MS = 5000; // revert to live sensing this long after last turn

// ---- GPS freshness (drone only, for FOLLOW HOME) ----
const unsigned long GPS_TIMEOUT_MS = 5000;

// ---- Proximity relay freshness (drone's RSSI report, for FOLLOW PUCK) ----
const unsigned long PROXIMITY_REPORT_TIMEOUT_MS = 800;

// =====================================================================
// GPS wire protocol (drone only - see esp_now_gps_tx.ino)
// =====================================================================
enum DeviceId : uint8_t { DEVICE_DRONE = 0, DEVICE_PUCK = 1 };

typedef struct struct_message {
  uint8_t deviceId;
  bool fixValid;
  double lat;
  double lon;
  uint8_t sats;
} struct_message;

struct_message droneMsg = { DEVICE_DRONE, false, 0, 0, 0 };
unsigned long droneLastSeen = 0;

// ---- Home point: captured once from the drone's first valid fix ----
bool homeSet = false;
double homeLat = 0.0;
double homeLon = 0.0;

// =====================================================================
// RSSI proximity relay (drone senses, this board just consumes it)
// =====================================================================
typedef struct {
  float rssi;
  uint8_t zone;
} proximity_report_t;

enum PuckZone { ZONE_NEAR = 0, ZONE_MID = 1, ZONE_FAR = 2, ZONE_LOST = 3 };
const char *zoneNames[] = { "NEAR", "MID", "FAR", "LOST" };

struct BeepPattern {
  uint16_t freq;      // tone frequency, Hz (0 = silent)
  uint16_t onMs;       // how long the tone sounds within each cycle
  uint16_t periodMs;   // full cycle length (on + off)
};
// Must match the beep feel esp_now_proximity_finder.ino uses, since the
// drone classifies zones with those same thresholds.
BeepPattern zonePatterns[4] = {
  { 2400, 110, 150 },   // NEAR - rapid, high pitched, almost continuous
  { 1500, 120, 400 },   // MID  - moderate beeping
  { 900,  150, 1000 },  // FAR  - slow, low-pitched beeps
  { 0,    0,   0 }      // LOST - silent
};

float latestRssi = 0;
int puckZone = ZONE_LOST;
unsigned long lastProximityReportMillis = 0;

// =====================================================================
// Mode state
// =====================================================================
enum FollowMode { MODE_FOLLOW_PUCK, MODE_FOLLOW_HOME };
FollowMode currentMode = MODE_FOLLOW_PUCK;

// ---- Debounced buttons (plain momentary press) ----
struct DebouncedButton {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
  DebouncedButton(uint8_t p) : pin(p), lastReading(HIGH), stableState(HIGH), lastChangeMs(0) {}
};
const unsigned long DEBOUNCE_MS = 40;

DebouncedButton puckBtn(BUTTON_FOLLOW_PUCK_PIN);
DebouncedButton homeBtn(BUTTON_FOLLOW_HOME_PIN);

// Returns true exactly once, on the press edge (HIGH -> LOW, since INPUT_PULLUP)
bool checkPressed(DebouncedButton &btn) {
  bool reading = digitalRead(btn.pin);
  if (reading != btn.lastReading) {
    btn.lastChangeMs = millis();
    btn.lastReading = reading;
  }
  bool pressedEdge = false;
  if (millis() - btn.lastChangeMs > DEBOUNCE_MS && reading != btn.stableState) {
    bool wasHigh = (btn.stableState == HIGH);
    btn.stableState = reading;
    if (wasHigh && reading == LOW) pressedEdge = true;
  }
  return pressedEdge;
}

// ---- Rotary encoder (quadrature, polled every loop - matches ui_test.ino) ----
volatile int8_t encDelta = 0;
uint8_t lastEncState = 0;
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void pollEncoder() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t idx = (lastEncState << 2) | state;
  encDelta += QUAD_TABLE[idx & 0x0F];
  lastEncState = state;
}

bool dialOverrideActive = false;
unsigned long lastDialMoveMs = 0;
float overrideDistanceM = 15.0f; // matches ui_test.ino's default approxDistanceM

void updateManualOverride() {
  if (encDelta != 0) {
    overrideDistanceM = constrain(overrideDistanceM + encDelta * ENCODER_STEP_M,
                                   OVERRIDE_DIST_MIN_M, OVERRIDE_DIST_MAX_M);
    encDelta = 0;
    dialOverrideActive = true;
    lastDialMoveMs = millis();
  }
  if (dialOverrideActive && millis() - lastDialMoveMs > DIAL_OVERRIDE_TIMEOUT_MS) {
    dialOverrideActive = false; // untouched long enough - hand control back to live sensing
  }
}

// =====================================================================
// FOLLOW HOME distance math
// =====================================================================
double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

float floatMap(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int distanceToFreqHz(float distM) {
  float clamped = constrain(distM, DIST_MIN_M, DIST_MAX_M);
  return (int)floatMap(clamped, DIST_MIN_M, DIST_MAX_M, FREQ_MAX_HZ, FREQ_MIN_HZ);
}

// For the OLED - not authoritative for audio (see currentTargetFreqHz)
bool activeDistanceValid = false;
float activeDistanceM = 0.0f;

// Single source of truth for "what frequency should be playing right now."
// Every mode (RSSI relay, home, manual override) funnels through here so
// updateToneAndLed() only ever has to deal with one number.
int currentTargetFreqHz() {
  unsigned long now = millis();

  if (dialOverrideActive) {
    activeDistanceValid = true;
    activeDistanceM = overrideDistanceM;
    return distanceToFreqHz(overrideDistanceM);
  }

  if (currentMode == MODE_FOLLOW_HOME) {
    bool droneFresh = droneMsg.fixValid && (now - droneLastSeen < GPS_TIMEOUT_MS);
    if (droneFresh && homeSet) {
      activeDistanceM = (float)haversineMeters(droneMsg.lat, droneMsg.lon, homeLat, homeLon);
      activeDistanceValid = true;
      return distanceToFreqHz(activeDistanceM);
    }
    activeDistanceValid = false;
    return 0;
  }

  // MODE_FOLLOW_PUCK - driven entirely by the drone's relayed RSSI zone.
  // No meters value here, so drive the same zone-based beep pattern
  // esp_now_proximity_finder.ino uses, gated on/off by phase.
  activeDistanceValid = false;
  bool reportFresh = (now - lastProximityReportMillis) <= PROXIMITY_REPORT_TIMEOUT_MS;
  int effectiveZone = reportFresh ? puckZone : ZONE_LOST;

  BeepPattern p = zonePatterns[effectiveZone];
  if (p.periodMs == 0) return 0; // ZONE_LOST
  uint32_t phase = now % p.periodMs;
  return (phase < p.onMs) ? p.freq : 0;
}

// =====================================================================
// Tone + LED, driven from one shared square wave so the LED is a
// literal mirror of the piezo's on/off cycle. Non-blocking.
// =====================================================================
bool waveState = false;
unsigned long lastToggleMicros = 0;

void updateToneAndLed(int freqHz) {
  if (freqHz <= 0) {
    digitalWrite(PIEZO_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    return;
  }
  unsigned long halfPeriodUs = 500000UL / (unsigned long)freqHz;
  if (micros() - lastToggleMicros >= halfPeriodUs) {
    lastToggleMicros = micros();
    waveState = !waveState;
    digitalWrite(PIEZO_PIN, waveState ? HIGH : LOW);
    digitalWrite(LED_PIN, waveState ? HIGH : LOW);
  }
}

// =====================================================================
// OLED (rate-limited so I2C traffic doesn't eat the loop). Placeholder
// content - swap out once the team settles on an actual screen design.
// =====================================================================
unsigned long lastDisplayUpdateMs = 0;
const unsigned long DISPLAY_INTERVAL_MS = 150;

void updateDisplay() {
  if (millis() - lastDisplayUpdateMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayUpdateMs = millis();

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println(currentMode == MODE_FOLLOW_PUCK ? "MODE: FOLLOW PUCK" : "MODE: FOLLOW HOME");

  if (dialOverrideActive) {
    display.println("SOURCE: MANUAL DIAL");
    display.printf("Dist: %.1f m\n", overrideDistanceM);
  } else if (currentMode == MODE_FOLLOW_HOME) {
    display.print("Drone: "); display.println(droneMsg.fixValid ? "fix" : "no fix");
    if (activeDistanceValid) display.printf("Dist: %.1f m\n", activeDistanceM);
    else display.println("Dist: -- (waiting)");
  } else {
    bool reportFresh = (millis() - lastProximityReportMillis) <= PROXIMITY_REPORT_TIMEOUT_MS;
    display.printf("Zone: %s\n", zoneNames[reportFresh ? puckZone : ZONE_LOST]);
    display.printf("RSSI: %.1f dBm\n", latestRssi);
    if (!reportFresh) display.println("(no relay from drone)");
  }

  display.display();
}

// =====================================================================
// ESP-NOW receive - dispatches on packet length since two message
// types (GPS struct_message, RSSI proximity_report_t) share one callback.
// =====================================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(struct_message)) {
    struct_message incoming;
    memcpy(&incoming, data, sizeof(incoming));

    if (incoming.deviceId == DEVICE_DRONE) {
      droneMsg = incoming;
      droneLastSeen = millis();
      if (!homeSet && droneMsg.fixValid) {
        homeLat = droneMsg.lat;
        homeLon = droneMsg.lon;
        homeSet = true;
        Serial.println("Home point captured from drone's first fix.");
      }
    }
    return;
  }

  if (len == sizeof(proximity_report_t)) {
    proximity_report_t incoming;
    memcpy(&incoming, data, sizeof(incoming));
    latestRssi = incoming.rssi;
    puckZone = incoming.zone;
    lastProximityReportMillis = millis();
    return;
  }

  Serial.printf("Unrecognized packet length: %d\n", len);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(BUTTON_FOLLOW_PUCK_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FOLLOW_HOME_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(PIEZO_PIN, LOW);

  lastEncState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("Display not found!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Controller booting...");
  display.display();

  // Fixed channel + TX power - must match esp_now_gps_tx.ino / puck_gps_tx.ino
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);

  Serial.print("Controller MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(put this address in the drone's peer list once known)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Controller ready.");
}

void loop() {
  pollEncoder();

  if (checkPressed(puckBtn)) currentMode = MODE_FOLLOW_PUCK;
  if (checkPressed(homeBtn)) currentMode = MODE_FOLLOW_HOME;

  updateManualOverride();

  int freqHz = currentTargetFreqHz();
  updateToneAndLed(freqHz);

  updateDisplay();
}
