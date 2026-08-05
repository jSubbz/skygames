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
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
const uint8_t TX_POWER_QUARTER_DBM = 84; // 84 = 21 dBm (max) - keep identical on all radios

const uint32_t BEACON_INTERVAL_MS = 100; // ~10 beacons/sec, matches esp_now_proximity_finder.ino
const uint32_t HEARTBEAT_INTERVAL_MS = 1000; // Serial print rate - flash-confirmation only, not a protocol change

typedef struct {
  uint32_t counter;
} beacon_t;

beacon_t outgoingBeacon = { 0 };

// Placeholder broadcast address - swap in the drone's real MAC later
uint8_t rxMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op on purpose - fires 10x/sec, would flood Serial otherwise
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Starting ESP-NOW RSSI beacon (puck)...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);

  Serial.print("This device's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(put this address in the drone's peer list once known)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Broadcasting beacons...");
  Serial.println("(heartbeat below confirms this build is alive - not part of the wire protocol)");
}

unsigned long lastSend = 0;
unsigned long lastHeartbeat = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= BEACON_INTERVAL_MS) {
    lastSend = now;
    outgoingBeacon.counter++;
    esp_now_send(rxMac, (uint8_t *)&outgoingBeacon, sizeof(outgoingBeacon));
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    Serial.printf("puck alive - beacon #%lu\n", (unsigned long)outgoingBeacon.counter);
  }
}
