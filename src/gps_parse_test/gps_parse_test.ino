/*
 * ESP32-C3 GPS UART Read + TinyGPS++ Parse Test
 * ------------------------------------------------
 * Same passive listener as gps_raw_test.ino, but now runs the
 * stream through TinyGPS++ and prints decoded values instead of
 * raw NMEA text. Still no ESP-NOW - just confirming the parser
 * sees valid data before adding radio complexity.
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
 * Open Serial Monitor at 115200. You'll see one status line per
 * second. "fix=0" with sats=0 is normal until it acquires - give
 * it clear sky view and a minute or two on a cold start.
 */

#include <Arduino.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN 20
#define GPS_BAUD   115200

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting GPS parse test...");

  // -1 for TX pin = RX only, never drives the tapped wire
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
}

unsigned long lastPrint = 0;

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.printf(
      "fix=%d  lat=%.6f  lon=%.6f  sats=%d  alt=%.1fm  spd=%.1fm/s  sentences=%lu  failedChecksum=%lu\n",
      gps.location.isValid(),
      gps.location.isValid() ? gps.location.lat() : 0.0,
      gps.location.isValid() ? gps.location.lng() : 0.0,
      gps.satellites.isValid() ? gps.satellites.value() : 0,
      gps.altitude.isValid() ? gps.altitude.meters() : 0.0f,
      gps.speed.isValid() ? gps.speed.mps() : 0.0f,
      gps.sentencesWithFix(),
      gps.failedChecksum()
    );

    // If this stays at 0 while you're clearly seeing NMEA text in
    // the raw test, it usually means TinyGPS++ isn't getting bytes
    // fast enough or characters are being dropped - but since your
    // raw test looked clean, that shouldn't be an issue here.
    if (gps.charsProcessed() < 10 && millis() > 5000) {
      Serial.println("WARNING: No NMEA data seen yet - check wiring.");
    }
  }
}
