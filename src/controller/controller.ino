/*
 * Controller / Receiver - consolidated draft (v4)
 * -------------------------------------------------
 * Replaces the separate blinky_rx / esp_now_display_rx / esp_now_gps_rx
 * drafts with one sketch.
 *
 * Architecture (as of this draft):
 *   Puck  --beacon_t, ~10x/sec-->  Drone
 *   Drone --filters RSSI, classifies NEAR/MID/FAR/LOST zone (same
 *           logic as esp_now_proximity_finder.ino's SEEKER role) -->
 *   Drone --proximity_report_t, ~10x/sec-->  Controller (this board)
 *   Controller drives buzzer/RGB LED off the zone the drone relays. No
 *   RSSI math happens here - the drone is the one flying around the
 *   puck, so it has to be the one sensing proximity; the controller
 *   just plays the result back to the operator.
 *
 *   Drone --struct_message (GPS), ~1x/sec--> Controller, for the
 *   separate FOLLOW HOME feature (drone <-> its own first fix, no
 *   controller GPS). Puck no longer sends GPS - drone<->puck finding
 *   is RSSI-only.
 *
 * Controls (2026-08-06 - button roles changed from earlier drafts):
 *   - Button 1 (MODE): toggles between FOLLOW PUCK and FOLLOW HOME each
 *     press - no longer two separate "set to X" buttons.
 *   - Button 2 (POT TOGGLE): enables/disables the potentiometer dial
 *     override entirely. Added because the dial's known worn-track
 *     jumpiness (see notes further down) could occasionally trip
 *     dialOverrideActive on its own and never let go of live sensing -
 *     this gives a hard way to shut the dial out of the loop while
 *     testing the real puck/drone system.
 *   - Potentiometer dial (only listened to while pot override is
 *     enabled): dev/test override. Turning it feeds a manual distance
 *     into the buzzer/LED instead of live sensing; reverts to live data
 *     automatically 5s after you stop turning it.
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
 * PIN CONFIG - confirmed against real controller wiring (2026-08-05).
 * ------------------------------------------------------------
 * Matches hardwaretest.ino, which is flashed on this board and fully
 * working: OLED on 22/23, buttons on 2/3, buzzer on 18, RGB LED
 * (common cathode) on 19/20/21, potentiometer wiper on GPIO0. The
 * earlier v3 draft's pins (I2C 6/7, buttons 4/5, rotary encoder
 * 2/3/1) were copied from ui_test.ino's separate bench rig and did
 * not match this board - replaced below.
 *
 * BUZZER - confirmed genuinely active (2026-08-05), not a passive
 * piezo. An active buzzer has its own fixed-pitch oscillator and can
 * only be switched on/off - it cannot be pitch-shifted by driving the
 * pin at different frequencies. So zone/distance feedback below is
 * conveyed by beep RATE (how fast it clicks) instead of pitch. Driven
 * via LEDC PWM rather than a plain digitalWrite so BUZZER_VOLUME_PERCENT
 * can turn the volume down (2026-08-05 - full HIGH/LOW drive was too loud).
 *
 * RGB LED (2026-08-06) - also PWM-driven now (not plain on/off), so
 * color is a genuine continuous gradient instead of a handful of fixed
 * hues: green = close, red = far, blending through yellow in between.
 * off = lost/no signal. (Earlier draft had this backwards - red=close -
 * flipped per team direction.) Still blinks in sync with the buzzer's
 * on/off click.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ---- Pin configuration (matches hardwaretest.ino / real wiring) ----
#define I2C_SDA 22                   // OLED SDA
#define I2C_SCL 23                   // OLED SCL
#define POT_PIN 0                    // potentiometer wiper - dev/test dial
#define BUTTON_MODE_PIN 2            // = hardwaretest.ino's BUTTON1_PIN - toggles FOLLOW PUCK/HOME
#define BUTTON_POT_TOGGLE_PIN 3      // = hardwaretest.ino's BUTTON2_PIN - enables/disables the dial
#define BUZZER_PIN 18                // active buzzer - on/off only, no pitch control
#define LED_R 19                     // RGB LED, common cathode
#define LED_G 20
#define LED_B 21

// ---- OLED config (same as esp_now_gps_rx.ino / ui_test.ino) ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1
// 0 = as-is, 2 = flipped 180 - set to whichever reads right-side up in
// your enclosure (bench video 2026-08-05 showed text upside down at 0).
#define DISPLAY_ROTATION 2
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---- Radio config - must match esp_now_gps_tx.ino / puck_gps_tx.ino ----
#define WIFI_CHANNEL 1
const uint8_t TX_POWER_QUARTER_DBM = 84; // 84 = 21 dBm (max) - keep identical on all radios

// ---- FOLLOW HOME distance -> beep-rate mapping. Tune once real-world
// distances are known. Buzzer is on/off only (see note above), so
// "getting warmer" is conveyed by faster clicking, not higher pitch. ----
const float DIST_MIN_M = 1.0f;             // at or below this: fastest click ("hot")
const float DIST_MAX_M = 100.0f;           // at or above this: slowest click ("cold")
const uint16_t HOME_CLICK_ON_MS = 90;      // click length, constant across the range
const uint16_t HOME_PERIOD_MIN_MS = 150;   // click period when close
const uint16_t HOME_PERIOD_MAX_MS = 1000;  // click period when far

// ---- Buzzer volume, via PWM duty cycle (see header note - active buzzer,
// full HIGH/LOW drive was reported too loud 2026-08-05). Lower duty =
// lower average voltage = quieter, down to some point where a given
// buzzer stops oscillating altogether - stay well above that floor. ----
const int BUZZER_PWM_FREQ_HZ = 20000;      // carrier above hearing range - no PWM whine
const int BUZZER_PWM_RESOLUTION_BITS = 8;  // 0-255 duty steps
const uint8_t BUZZER_VOLUME_PERCENT = 65;  // tune this one knob if it needs to go up/down
const uint8_t BUZZER_DUTY_ON = (uint8_t)(BUZZER_VOLUME_PERCENT * 255 / 100);

// ---- RGB LED gradient config (2026-08-06). Color is continuous now, not
// bucketed - FOLLOW HOME/dial reuse DIST_MIN_M/DIST_MAX_M directly as the
// gradient's near/far ends. FOLLOW PUCK doesn't get a meters value from
// the drone, so it mirrors the RSSI thresholds instead (cosmetic only -
// not wire-protocol critical, but keep roughly in sync with
// esp_now_gps_tx.ino's RSSI_NEAR/RSSI_FAR if those get retuned). ----
const float RSSI_NEAR_MIRROR_DBM = -55.0f; // at/stronger than this: full green
const float RSSI_FAR_MIRROR_DBM  = -82.0f; // at/weaker than this: full red

// ---- Manual override (potentiometer) config ----
const int POT_ADC_MIN = 0;
const int POT_ADC_MAX = 4095;                // ESP32 12-bit ADC
const int POT_MOVE_THRESHOLD = 25;           // ignore ADC jitter smaller than this
const float POT_SMOOTHING_ALPHA = 0.25f;     // 0-1, lower = smoother. Without this, raw ADC
                                              // noise fed straight into periodMs every loop and
                                              // made the buzzer crackle instead of click cleanly
                                              // while turning fast (seen in 2026-08-05 bench video)
const float OVERRIDE_DIST_MIN_M = 0.0f;
const float OVERRIDE_DIST_MAX_M = DIST_MAX_M;
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

// on/off click timing only - no freq field, since the buzzer can't be
// pitch-shifted. Must still *feel* like the beep feel
// esp_now_proximity_finder.ino uses (rapid when close, slow when far).
struct BeepPattern {
  uint16_t onMs;       // how long the buzzer sounds within each cycle
  uint16_t periodMs;   // full cycle length (on + off); 0 = silent
};
BeepPattern zonePatterns[4] = {
  { 110, 150 },    // NEAR - rapid, almost continuous
  { 120, 400 },    // MID  - moderate beeping
  { 150, 1000 },   // FAR  - slow beeps
  { 0,   0    }    // LOST - silent
};

// PWM intensities (0-255 per channel) now, not on/off bools - lets the
// LED show a real blended color instead of one of 8 fixed hues.
struct RgbColor { uint8_t r, g, b; };
const RgbColor COLOR_OFF = { 0, 0, 0 };

// All struct types live up here, before ANY function definition (even
// ones like gradientColor() right below that don't use these two) - the
// Arduino IDE hoists auto-generated prototypes for every function as one
// block, inserted at the position of the FIRST function definition in the
// file. Whatever that first function ends up being, every type used by
// ANY function's signature has to already be defined above it, or you
// get "does not name a type" (bit us twice now - 2026-08-05, 2026-08-06 -
// each time a new function got added earlier in the file than the last
// one). Keeping every struct in one block up front avoids this for good.
struct AvState {
  BeepPattern pattern;
  RgbColor color;
};

struct DebouncedButton {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
  DebouncedButton(uint8_t p) : pin(p), lastReading(HIGH), stableState(HIGH), lastChangeMs(0) {}
};

// t=0 -> full green (close), t=1 -> full red (far), blending through
// yellow at t=0.5. Caller is responsible for mapping their real-world
// value (meters or RSSI) onto this normalized 0..1 range first.
RgbColor gradientColor(float t) {
  t = constrain(t, 0.0f, 1.0f);
  uint8_t r, g;
  if (t < 0.5f) {
    r = (uint8_t)(t * 2.0f * 255.0f);
    g = 255;
  } else {
    r = 255;
    g = (uint8_t)((1.0f - t) * 2.0f * 255.0f);
  }
  return { r, g, 0 };
}

float rssiToGradientT(float rssiDbm) {
  float clamped = constrain(rssiDbm, RSSI_FAR_MIRROR_DBM, RSSI_NEAR_MIRROR_DBM);
  // near (less negative) -> t=0, far (more negative) -> t=1
  return (RSSI_NEAR_MIRROR_DBM - clamped) / (RSSI_NEAR_MIRROR_DBM - RSSI_FAR_MIRROR_DBM);
}

float latestRssi = 0;
int puckZone = ZONE_LOST;
unsigned long lastProximityReportMillis = 0;

// =====================================================================
// Mode state
// =====================================================================
enum FollowMode { MODE_FOLLOW_PUCK, MODE_FOLLOW_HOME };
FollowMode currentMode = MODE_FOLLOW_PUCK;

// ---- Debounced buttons (plain momentary press; struct itself is defined
// up top with the other types - see note there) ----
const unsigned long DEBOUNCE_MS = 40;

DebouncedButton modeBtn(BUTTON_MODE_PIN);
DebouncedButton potToggleBtn(BUTTON_POT_TOGGLE_PIN);

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

// ---- Manual override (potentiometer, absolute reading - no encoder on
// this board). Turning the dial is detected as "reading moved more than
// jitter allows"; the override then tracks the dial's absolute position
// live, and hands control back to live sensing 5s after it stops moving.
// Button 2 (BUTTON_POT_TOGGLE_PIN) can shut this whole thing off - see
// potOverrideEnabled below - since the pot's worn track (further down)
// could occasionally fake a "touch" and hijack control on its own. ----
bool potOverrideEnabled = true; // toggled by Button 2 - see loop()
bool dialOverrideActive = false;
unsigned long lastDialMoveMs = 0;
float overrideDistanceM = 15.0f;
int lastPotReading = -1;
float smoothedPot = -1.0f;

// Real potentiometers/wiring rarely hit the full theoretical 0-4095 ADC
// swing POT_ADC_MIN/MAX assume - mapping against a range the pot can't
// actually reach compresses the usable output down to a fraction of
// OVERRIDE_DIST_MAX_M (this is why it only felt like ~1-2m of travel).
// So instead of trusting POT_ADC_MIN/MAX, track the widest raw range
// actually seen and map against that - a couple of full turns end-to-end
// after boot and it self-calibrates to whatever this pot really does.
int potObservedMin = POT_ADC_MAX;
int potObservedMax = POT_ADC_MIN;

unsigned long lastDialDebugMs = 0;
const unsigned long DIAL_DEBUG_INTERVAL_MS = 500; // Serial visibility while calibrating

// Median-of-N glitch filter, applied before anything else touches the
// reading. Bench test (2026-08-05) showed both physical end-stops read
// cleanly but the middle of the travel would suddenly jump to an
// unrelated value (e.g. 14m -> 35m -> 41m -> 14m while turning one way)
// - classic symptom of a worn/dirty spot on the pot's resistive track.
// A median rejects a single bad sample outright instead of just
// dampening it like the EMA below does; the EMA still runs after this
// for general smoothness. If glitches persist even with this in place,
// it's likely a real dead spot on the track - try working the dial back
// and forth across its full range repeatedly (can wipe a dirty contact
// clean), or swap the pot if it keeps happening.
const int POT_MEDIAN_WINDOW = 5; // odd, so there's always a clean middle value
int potHistory[POT_MEDIAN_WINDOW];
int potHistoryCount = 0;
int potHistoryIdx = 0;

int medianOfPotHistory() {
  int sorted[POT_MEDIAN_WINDOW];
  int n = potHistoryCount;
  memcpy(sorted, potHistory, n * sizeof(int));
  for (int i = 1; i < n; i++) {
    int key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[n / 2];
}

float floatMap(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void updateManualOverride() {
  if (!potOverrideEnabled) {
    dialOverrideActive = false; // force live sensing while the dial is switched off
    return;                     // don't even read the pot - keeps it fully out of the loop
  }

  int rawPot = analogRead(POT_PIN);
  potHistory[potHistoryIdx] = rawPot;
  potHistoryIdx = (potHistoryIdx + 1) % POT_MEDIAN_WINDOW;
  if (potHistoryCount < POT_MEDIAN_WINDOW) potHistoryCount++;
  int pot = medianOfPotHistory(); // de-glitched reading - use this below, not rawPot

  if (lastPotReading < 0) lastPotReading = pot;   // first read - nothing to compare yet
  if (smoothedPot < 0) smoothedPot = pot;         // seed the filter

  // Movement detection uses the de-glitched (median) reading too now, so a
  // single bad sample can't falsely trigger/extend the override.
  if (abs(pot - lastPotReading) > POT_MOVE_THRESHOLD) {
    dialOverrideActive = true;
    lastDialMoveMs = millis();
  }
  lastPotReading = pot;

  if (pot < potObservedMin) potObservedMin = pot;
  if (pot > potObservedMax) potObservedMax = pot;

  // But the value that actually drives periodMs is smoothed - see
  // POT_SMOOTHING_ALPHA above for why.
  smoothedPot = POT_SMOOTHING_ALPHA * pot + (1.0f - POT_SMOOTHING_ALPHA) * smoothedPot;

  if (dialOverrideActive) {
    int rangeMax = max(potObservedMax, potObservedMin + 1); // guard div-by-zero pre-calibration
    overrideDistanceM = floatMap(smoothedPot, potObservedMin, rangeMax,
                                  OVERRIDE_DIST_MIN_M, OVERRIDE_DIST_MAX_M);
    if (millis() - lastDialMoveMs > DIAL_OVERRIDE_TIMEOUT_MS) {
      dialOverrideActive = false; // untouched long enough - hand control back to live sensing
    }

    if (millis() - lastDialDebugMs >= DIAL_DEBUG_INTERVAL_MS) {
      lastDialDebugMs = millis();
      Serial.printf("DIAL raw=%d  med=%d  observed=[%d,%d]  dist=%.1fm\n",
                     rawPot, pot, potObservedMin, potObservedMax, overrideDistanceM);
    }
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

uint16_t distanceToPeriodMs(float distM) {
  float clamped = constrain(distM, DIST_MIN_M, DIST_MAX_M);
  return (uint16_t)floatMap(clamped, DIST_MIN_M, DIST_MAX_M, HOME_PERIOD_MIN_MS, HOME_PERIOD_MAX_MS);
}

RgbColor homeDistanceColor(float distM) {
  float clamped = constrain(distM, DIST_MIN_M, DIST_MAX_M);
  float t = (clamped - DIST_MIN_M) / (DIST_MAX_M - DIST_MIN_M); // 0=close/green, 1=far/red
  return gradientColor(t);
}

// For the OLED - not authoritative for audio (see currentAvState)
bool activeDistanceValid = false;
float activeDistanceM = 0.0f;

// Single source of truth for "what should the buzzer/LED be doing right
// now." Every mode (RSSI relay, home, manual override) funnels through
// here so updateBuzzerAndLed() only ever has to deal with one state.
AvState currentAvState() {
  unsigned long now = millis();

  if (dialOverrideActive) {
    activeDistanceValid = true;
    activeDistanceM = overrideDistanceM;
    BeepPattern p = { HOME_CLICK_ON_MS, distanceToPeriodMs(overrideDistanceM) };
    return { p, homeDistanceColor(overrideDistanceM) };
  }

  if (currentMode == MODE_FOLLOW_HOME) {
    bool droneFresh = droneMsg.fixValid && (now - droneLastSeen < GPS_TIMEOUT_MS);
    if (droneFresh && homeSet) {
      activeDistanceM = (float)haversineMeters(droneMsg.lat, droneMsg.lon, homeLat, homeLon);
      activeDistanceValid = true;
      BeepPattern p = { HOME_CLICK_ON_MS, distanceToPeriodMs(activeDistanceM) };
      return { p, homeDistanceColor(activeDistanceM) };
    }
    activeDistanceValid = false;
    return { { 0, 0 }, COLOR_OFF };
  }

  // MODE_FOLLOW_PUCK - driven by the drone's relayed RSSI zone (beep rate,
  // still discrete/bucketed - fine, it doesn't need to be continuous) and
  // the raw relayed RSSI value (LED color - continuous gradient).
  activeDistanceValid = false;
  bool reportFresh = (now - lastProximityReportMillis) <= PROXIMITY_REPORT_TIMEOUT_MS;
  int effectiveZone = reportFresh ? puckZone : ZONE_LOST;
  RgbColor puckColor = (effectiveZone == ZONE_LOST) ? COLOR_OFF
                                                     : gradientColor(rssiToGradientT(latestRssi));
  return { zonePatterns[effectiveZone], puckColor };
}

// =====================================================================
// Buzzer + RGB LED, driven from one shared on/off cycle so the LED is
// a color-coded mirror of the buzzer's click. Non-blocking.
// =====================================================================
void setLed(const RgbColor &c) {
  ledcWrite(LED_R, c.r);
  ledcWrite(LED_G, c.g);
  ledcWrite(LED_B, c.b);
}

void updateBuzzerAndLed(const AvState &state) {
  bool on = false;
  if (state.pattern.periodMs > 0) {
    uint32_t phase = millis() % state.pattern.periodMs;
    on = phase < state.pattern.onMs;
  }
  ledcWrite(BUZZER_PIN, on ? BUZZER_DUTY_ON : 0);
  setLed(on ? state.color : COLOR_OFF);
}

// =====================================================================
// OLED (rate-limited so I2C traffic doesn't eat the loop). Layout
// borrowed from ui_test.ino's "SET PUCK" mockup (header bar, big
// readout, crosshair icon, small label, footer bar) per team request
// 2026-08-06, split into two screens: DEV MODE while the dial is
// actively being worked (dialOverrideActive), PLAY MODE otherwise -
// live sensing. DEV MODE can only happen while the pot is enabled (see
// potOverrideEnabled - updateManualOverride() can't set
// dialOverrideActive while it's off), so only PLAY MODE's header needs
// to flag a disabled pot.
//
// Dropped the raw RSSI dBm readout the old screen had, to match
// ui_test's cleaner look - easy to add back (e.g. into DEV MODE, or a
// long-press reveal) if it turns out to be missed for threshold tuning.
// =====================================================================
unsigned long lastDisplayUpdateMs = 0;
const unsigned long DISPLAY_INTERVAL_MS = 150;

void drawHeader(const char *label) {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(label);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);
}

void drawCrosshair(int cx, int cy) {
  display.drawCircle(cx, cy, 12, SH110X_WHITE);
  display.drawCircle(cx, cy, 2, SH110X_WHITE);
}

void drawFooter() {
  display.drawFastHLine(0, 54, SCREEN_WIDTH, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 57);
  display.print("1:MODE  2:POT");
}

void drawDevModeScreen() {
  drawHeader("DEV MODE");

  display.setTextSize(2);
  display.setCursor(4, 20);
  display.print("~");
  display.print(overrideDistanceM, 1);
  display.print(" m");

  drawCrosshair(108, 32);

  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("SOURCE: MANUAL DIAL");

  drawFooter();
}

void drawPlayModeScreen() {
  drawHeader(potOverrideEnabled ? "PLAY MODE" : "PLAY MODE (POT OFF)");

  display.setTextSize(2);
  display.setCursor(4, 20);
  if (currentMode == MODE_FOLLOW_HOME) {
    if (activeDistanceValid) {
      display.print("~");
      display.print(activeDistanceM, 1);
      display.print(" m");
    } else {
      display.print("NO FIX");
    }
  } else {
    bool reportFresh = (millis() - lastProximityReportMillis) <= PROXIMITY_REPORT_TIMEOUT_MS;
    display.print(zoneNames[reportFresh ? puckZone : ZONE_LOST]);
  }

  drawCrosshair(108, 32);

  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print(currentMode == MODE_FOLLOW_PUCK ? "FOLLOW PUCK" : "FOLLOW HOME");

  drawFooter();
}

void updateDisplay() {
  if (millis() - lastDisplayUpdateMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayUpdateMs = millis();

  display.clearDisplay();
  if (dialOverrideActive) {
    drawDevModeScreen();
  } else {
    drawPlayModeScreen();
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

  // LEDC PWM (not plain digitalWrite) for the LED channels and buzzer -
  // needed for the color gradient and the volume knob, see notes above.
  ledcAttach(LED_R, 5000, 8);
  ledcAttach(LED_G, 5000, 8);
  ledcAttach(LED_B, 5000, 8);
  ledcAttach(BUZZER_PIN, BUZZER_PWM_FREQ_HZ, BUZZER_PWM_RESOLUTION_BITS);
  pinMode(BUTTON_MODE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_POT_TOGGLE_PIN, INPUT_PULLUP);

  setLed(COLOR_OFF);
  ledcWrite(BUZZER_PIN, 0);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("Display not found!");
    while (1);
  }
  display.setRotation(DISPLAY_ROTATION);
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
  if (checkPressed(modeBtn)) {
    currentMode = (currentMode == MODE_FOLLOW_PUCK) ? MODE_FOLLOW_HOME : MODE_FOLLOW_PUCK;
  }
  if (checkPressed(potToggleBtn)) {
    potOverrideEnabled = !potOverrideEnabled;
    Serial.printf("Pot override %s\n", potOverrideEnabled ? "ENABLED" : "DISABLED");
  }

  updateManualOverride();

  AvState state = currentAvState();
  updateBuzzerAndLed(state);

  updateDisplay();
}
