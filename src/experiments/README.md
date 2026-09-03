# Experiments

Bench sketches from building the system, each one written to prove a single
component or idea on its own before it was allowed near the real thing. None of
them are part of the game; they are kept because they are how the three sketches
in `src/` came to work, and because several are still the quickest way to test a
board you have just soldered.

| Sketch | What it proved |
|---|---|
| `mac-finder` | Prints a board's MAC address. Needed before ESP-NOW can address a specific peer. |
| `tesst` | Blinks the onboard LED. The first thing to run on a board you are not sure is alive. |
| `blinky_tx` / `blinky_rx` | Smallest possible ESP-NOW link: one board makes the other's LED blink. Proves the radio pairing before any payload matters. |
| `hardwaretest` | ESP32-C6 Pico bring-up — every pin on the controller exercised in one sketch. |
| `gps_parse_test` | Reads the GPS module over UART and parses it with TinyGPS++. Passive listener, no radio. |
| `esp_now_display_tx` / `esp_now_display_rx` | Sends an invented, slowly drifting GPS fix to an SH1106 OLED. Let the display and the message format be tested with no GPS lock and no second location. |
| `esp_now_gps_rx` | The display receiver wired to real GPS data rather than the mock. |
| `esp_now_proximity_finder` | Brayden's two-role prototype: one board broadcasts a beacon, the other reads RSSI, filters it and buckets it into NEAR / MID / FAR / LOST. This is the sketch the whole RSSI approach came out of, and both `puck_gps_tx` and the drone's sensing are descended from it. |
| `ui_test` | Standalone rig for the "Set Puck" screen — OLED, two buttons and a rotary encoder, with no radio attached. |

## Flashing any of them

Arduino IDE with the `esp32` boards package by Espressif, version 3.x. Board
depends on the sketch — the ESP32-S3 Zero sketches want **ESP32S3 Dev Module**
with *USB CDC On Boot* enabled, which is what makes Serial work over the native
USB-C port. Individual sketches note their own wiring in the header comment.
