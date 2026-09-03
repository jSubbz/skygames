# Skygames

Hide and seek played with an FPV drone. A puck is hidden somewhere in the play
area; the pilot flies out to find it, and a handheld controller gets warmer or
colder as the drone closes in.

![The controller and the puck. The controller is a printed hinged case with an OLED reading "DEV MODE", a lit red LED, two buttons and a dial; the puck is a black printed disc.](docs/controller-and-puck.jpg)

## How the game works

Three boards, each with one job.

The **puck** does nothing but shout. It broadcasts a small beacon packet about
ten times a second and never listens for anything.

The **drone** is the seeker. It hears the puck's beacon, reads the signal
strength of each packet as it arrives, filters the noise out and sorts the
result into one of four zones — near, mid, far, or lost. It sends that verdict
to the controller ten times a second.

The **controller** plays the verdict back to the pilot through a buzzer and an
RGB LED, and does no sensing of its own.

Putting the sensing on the drone is the whole trick: the drone is the thing
actually moving around the puck, so it is the only board whose distance to the
puck means anything. A controller standing still on the ground would measure the
pilot's distance to the puck, which nobody needs to know.

## Finding by signal strength, not GPS

The original design had the puck and the drone both report GPS positions and the
controller work out the distance between them. It was abandoned because the
M10Q module could not hold a fix indoors, and the system had to work indoors and
out. Signal strength needs no satellites, and while it is noisy it is monotonic
— which is all a hot-and-cold game actually requires.

GPS survives in one place: **follow home**, where the drone measures against its
own launch point rather than against the puck. That distance is large enough for
GPS to be good at it, and it is an outdoor feature anyway.

![The inside of the puck: a printed disc with an ESP32-C3 board screwed to it, sitting on a page of wiring notes.](docs/puck-internals.jpg)

## Feedback

The buzzer is an active one, so it has a fixed pitch and can only be switched on
and off. Proximity is therefore carried by **beep rate** rather than pitch —
faster clicks as you close in. It is driven through PWM rather than a plain
digital write so the volume can be turned down, which it needed to be.

The RGB LED is also PWM-driven and runs a continuous gradient rather than a
handful of fixed colours: green when close, red when far, blending through
yellow between, dark when the signal is lost. It blinks in step with the buzzer.

Two buttons: **mode** toggles between follow-puck and follow-home, and **pot
toggle** enables or disables the potentiometer dial, which is a bench override
that feeds a manual distance in place of live sensing.

## Repository layout

```
src/controller/        the handheld unit — display, buzzer, RGB LED, buttons
src/esp_now_gps_tx/    the drone — GPS fix, RSSI sensing, relays both
src/puck_gps_tx/       the puck — beacon broadcaster
src/experiments/       bench sketches, one per component (see its README)
models/                FreeCAD enclosures for the puck and the controller
```

Those three sketches replaced eleven separate ones. Every component was proved
on its own first — a sketch that only blinks an LED over the radio, a sketch
that only parses GPS, a sketch that only drives the screen — and the working
parts were then folded into three programs that talk to each other. The
originals are kept under `src/experiments/`, both as a record and because
several are still the fastest way to test a board you have just soldered.

`puck_gps_tx` keeps its name from when the puck still had GPS. The name is
wrong and the history is clearer with it left alone.

## Hardware

| Part | Board |
|---|---|
| Controller | ESP32-C6 Pico |
| Drone | ESP32-C3 |
| Puck | ESP32-C3 |

| Peripheral | Notes |
|---|---|
| SH1106 OLED | I2C, driven through Adafruit GFX and SH110X |
| M10Q GPS module | UART, parsed with TinyGPS++ |
| Active piezo buzzer | Fixed pitch, so feedback is by rate |
| Common-cathode RGB LED | PWM, continuous green-to-red gradient |
| Momentary pushbuttons | Mode, and the dial override |
| Potentiometer | Bench override for manual distance |
| 18650 cell + TP4056 | Power and USB-C charging |

Controller wiring, confirmed against the built unit:

```
OLED (I2C)        SDA 22, SCL 23
Buttons           mode 2, pot toggle 3
Buzzer            18
RGB LED           19 / 20 / 21
Potentiometer     0
```

On the drone, the GPS module's TX line goes to GPIO20. That line is read-only —
the sketch passes -1 for its own TX pin so it can never drive a wire it is only
tapping.

## Flashing

Arduino IDE with the `esp32` boards package by Espressif, version 3.x. For the
ESP32-S3 boards, choose **ESP32S3 Dev Module** and enable *USB CDC On Boot*,
which is what makes Serial work over the native USB-C port.

Libraries: **TinyGPSPlus** by Mikal Hart (not the older `TinyGPS`, which has a
different API), **Adafruit GFX**, **Adafruit SH110X**.

All three boards must be locked to the same WiFi channel — see `WIFI_CHANNEL`
in each sketch. Signal strength readings are only comparable within a channel,
and the link is only reliable when the radios agree.

![A drone frame being soldered by hand, motors and wiring visible.](docs/drone-assembly.jpg)

## Enclosures

`models/` holds FreeCAD sources for the puck and controller housings. There are
no mesh exports yet, so printing one currently means opening FreeCAD.

## Known rough edges

The boards address each other by **broadcast**, not by paired MAC. It works with
no setup, and it means a board will also pick up beacons from any other Skygames
puck in range. Each sketch prints its own MAC on boot, so swapping in real peer
addresses is a small change when it matters.

The controller's **screen content is a placeholder** — a plain status readout,
not a designed interface. The "set puck" screen concept in
`src/experiments/ui_test` is where that was heading.

The **potentiometer has a worn track** and can jump on its own. That is why
there is a button to shut the dial out of the loop entirely.

## Where it was going

Three directions were planned and not built. **More game modes** — capture the
flag, push the cart — on the same telemetry and RSSI foundation, since the game
logic is centralised on the controller and adding a mode needs no drone firmware
change. **A drone-agnostic mount**, built around the 1S/18650 power system that
consumer micro-drones already use, so the electronics could be added to an
existing drone rather than demanding a custom build. And the commercial side.

![Early bench work: an OLED, an encoder and a bare ESP32 board on a desk beside a notebook sketch of the controller.](docs/bench-design.jpg)

## Credits

Built with Brayden ([@braybuch](https://github.com/braybuch)) for ENL 4003.
Brayden did the hardware selection, bench testing and enclosure work, and wrote
the original proximity finder that the whole signal-strength approach came out
of. The commit history is his as much as mine.

## Licence

MIT — see [LICENSE](LICENSE).
