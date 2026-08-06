/*
 * PUCK - ESP-NOW RSSI beacon broadcaster
 * -----------------------------------------------------------------
 * The puck's only job now: broadcast a small beacon packet ~10x/sec
 * so the drone (esp_now_gps_tx.ino) can read this packet's RSSI on
 * arrival, filter it, and classify proximity into a NEAR/MID/FAR/LOST
 * zone. Same beacon_t + timing as Brayden's esp_now_proximity_finder.ino
 * HIDER role - this sketch is effectively that role, dedicated (no
 * GPS, no role switch needed since this board is always the puck).
 *
 * GPS was dropped from the puck on purpose: drone<->puck finding is
 * RSSI-only now, so a GPS fix here had no consumer. FOLLOW HOME still
 * works fine without it - that's purely a drone<->captured-home-point
 * calculation.
 *
 * Filename kept as puck_gps_tx.ino for continuity with the rest of
 * the repo/history, even though it no longer touches GPS.
 *
 * WIRE PROTOCOL: beacon_t must match esp_now_gps_tx.ino (drone) and
 * esp_now_proximity_finder.ino exactly:
 *
 *   typedef struct { uint32_t counter; } beacon_t;
 *
 * Fixed WiFi channel below must match esp_now_gps_tx.ino / controller.ino.
 *
 * Update rxMac[] below with the drone's real MAC address once you
 * know it (the drone prints its own MAC to Serial on boot). 0xFF
 * broadcast works for now without any pairing.
 *
 * FLASH-READY (2026-08-05): this is the build to flash over whatever
 * is currently on the puck if you can't confirm what that is. Added a
 * ~1x/sec heartbeat print below (see HEARTBEAT_INTERVAL_MS) purely so
 * you can open Serial Monitor after flashing and see the beacon
 * counter ticking up as live confirmation this exact build is running
 * - it changes no wire-protocol behavior.
 *
 * STATUS LED (2026-08-06): board's onboard LED is GPIO8, active-HIGH -
 * confirmed from blinky_rx.ino, which was previously run on this same
 * hardware. Flashes briefly on every beacon send (~10x/sec) as a purely
 * visual "yes, this is actually transmitting" indicator - no protocol
 * effect, same spirit as the Serial heartbeat above.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
const uint8_t TX_POWER_QUARTER_DBM = 84; // 84 = 21 dBm (max) - keep identical on all radios

const uint32_t BEACON_INTERVAL_MS = 100; // ~10 beacons/sec, matches esp_now_proximity_finder.ino
const uint32_t HEARTBEAT_INTERVAL_MS = 1000; // Serial print rate - flash-confirmation only, not a protocol change

#define LED_PIN 8                   // onboard LED, active-HIGH (see blinky_rx.ino)
const uint32_t LED_PULSE_MS = 15;   // brief flash per send - short enough not to look "stuck on" at 10Hz

typedef struct {
  uint32_t counter;
} beacon_t;

beacon_t outgoingBeacon = { 0 };

// Placeholder broadcast address - swap in the drone's real MAC later
uint8_t rxMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op on purpose - fires 10x/sec, would flood Serial otherwise
}

// Debug checkpoint helper (2026-08-06) - forces the print out immediately
// with a flush + tiny delay, so if something crashes right after a
// checkpoint, that checkpoint's text still has a chance to actually reach
// the monitor instead of sitting lost in an unflushed buffer. Temporary,
// for tracking down why this board goes totally silent on this sketch
// (works fine on a plain no-WiFi blink test) - remove once found.
void checkpoint(const char *label) {
  Serial.print("CP: ");
  Serial.println(label);
  Serial.flush();
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  checkpoint("serial up");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  checkpoint("LED pin set");

  WiFi.mode(WIFI_STA);
  checkpoint("WiFi.mode(WIFI_STA) done");

  WiFi.disconnect();
  checkpoint("WiFi.disconnect() done");

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  checkpoint("esp_wifi_set_channel() done");

  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);
  checkpoint("esp_wifi_set_max_tx_power() done");

  Serial.print("This device's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.flush();
  checkpoint("got MAC address");

  esp_err_t initResult = esp_now_init();
  checkpoint(initResult == ESP_OK ? "esp_now_init() OK" : "esp_now_init() FAILED");
  if (initResult != ESP_OK) {
    return;
  }

  esp_now_register_send_cb(onDataSent);
  checkpoint("send callback registered");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  esp_err_t peerResult = esp_now_add_peer(&peerInfo);
  checkpoint(peerResult == ESP_OK ? "peer added OK" : "add_peer FAILED");
  if (peerResult != ESP_OK) {
    return;
  }

  Serial.println("Broadcasting beacons...");
  Serial.println("(heartbeat below confirms this build is alive - not part of the wire protocol)");
  Serial.flush();
}

unsigned long lastSend = 0;
unsigned long lastHeartbeat = 0;
unsigned long ledOffAtMs = 0;
bool ledIsOn = false;

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= BEACON_INTERVAL_MS) {
    lastSend = now;
    outgoingBeacon.counter++;
    esp_now_send(rxMac, (uint8_t *)&outgoingBeacon, sizeof(outgoingBeacon));

    digitalWrite(LED_PIN, HIGH);
    ledIsOn = true;
    ledOffAtMs = now + LED_PULSE_MS;
  }

  if (ledIsOn && now >= ledOffAtMs) {
    digitalWrite(LED_PIN, LOW);
    ledIsOn = false;
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    Serial.printf("puck alive - beacon #%lu\n", (unsigned long)outgoingBeacon.counter);
  }
}
