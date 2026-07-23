// Hide & Seek Drone - "Set Puck" Screen UI Test
// -----------------------------------------------
// Quick standalone test rig for the SH1106 OLED + 2 buttons + rotary encoder.
// No ESP-NOW / radio here on purpose -- this is just to dial in the UI feel.
// Reused the display init/wiring style from the ESP-NOW receiver script.
//
// Wiring (ESP32-C6 Pico, change to match your board):
//   OLED SDA -> GPIO6, SCL -> GPIO7   (SH1106, 128x64, addr 0x3C)
//   Button A (CONFIRM / SET PUCK) -> GPIO4  (to GND, INPUT_PULLUP)
//   Button B (CANCEL / BACK)      -> GPIO5  (to GND, INPUT_PULLUP)
//   Encoder A                     -> GPIO2
//   Encoder B                     -> GPIO3
//   Encoder push (optional)       -> GPIO1  (to GND, INPUT_PULLUP)
//
// Turning the encoder adjusts a simulated "approx distance to puck" reading
// so you can see how the number/layout looks live. Button A "confirms" the
// puck placement, Button B cancels back out. All fake data, just for
// checking the screen layout, font sizes, and landscape orientation.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ---------- Pins ----------
#define I2C_SDA   6
#define I2C_SCL   7

#define BTN_CONFIRM 4
#define BTN_CANCEL  5

#define ENC_A       2
#define ENC_B       3
#define ENC_SW      1   // optional encoder push button

// ---------- Display ----------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
#define OLED_RESET    -1

// 0 = landscape as-is (128 wide x 64 tall, default for this panel)
// 2 = landscape flipped 180 (if your OLED is mounted upside down)
#define DISPLAY_ROTATION 0

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- App state ----------
enum UiState { SET_PUCK, CONFIRMED, CANCELLED };
UiState uiState = SET_PUCK;
unsigned long stateEnteredAt = 0;

// Simulated puck data (stand-ins for the real GPS/ranging values later)
float approxDistanceM = 15.0f;
int   headingDeg = 0; // just for a little visual life on the icon

// ---------- Button debouncing ----------
struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
};

Button btnConfirm = { BTN_CONFIRM, HIGH, HIGH, 0 };
Button btnCancel  = { BTN_CANCEL,  HIGH, HIGH, 0 };
const unsigned long DEBOUNCE_MS = 25;

bool buttonPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastChangeMs = millis();
  }
  bool pressedEdge = false;
  if ((millis() - b.lastChangeMs) > DEBOUNCE_MS) {
    if (reading != b.stableState) {
      b.stableState = reading;
      if (b.stableState == LOW) { // active-low, pull-up
        pressedEdge = true;
      }
    }
  }
  b.lastReading = reading;
  return pressedEdge;
}

// ---------- Encoder (simple quadrature, polled) ----------
volatile int8_t encDelta = 0;
uint8_t lastEncState = 0;

// Standard 4-step quadrature transition table
void pollEncoder() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t state = (a << 1) | b;

  static const int8_t table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  uint8_t idx = (lastEncState << 2) | state;
  encDelta += table[idx & 0x0F];
  lastEncState = state;
}

// ---------- UI drawing ----------
void drawHeader(const char *label) {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(label);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);
}

void drawPuckIcon(int cx, int cy) {
  // small crosshair "puck marker" icon, rotates a tick mark for a bit of life
  display.drawCircle(cx, cy, 12, SH110X_WHITE);
  display.drawCircle(cx, cy, 2, SH110X_WHITE);
  float rad = headingDeg * PI / 180.0f;
  int x2 = cx + (int)(12 * cos(rad));
  int y2 = cy + (int)(12 * sin(rad));
  display.drawLine(cx, cy, x2, y2, SH110X_WHITE);
}

void drawSetPuckScreen() {
  display.clearDisplay();
  drawHeader("SET PUCK");

  // Distance readout, mid-size so it stays readable but text stays compact
  display.setTextSize(2);
  display.setCursor(4, 20);
  display.print("~");
  display.print(approxDistanceM, 1);
  display.print(" m");

  // small icon on the right side, landscape layout
  drawPuckIcon(108, 32);

  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("approx. distance");

  display.drawFastHLine(0, 54, SCREEN_WIDTH, SH110X_WHITE);
  display.setCursor(0, 57);
  display.print("A:SET  B:BACK  DIAL:ADJ");

  display.display();
}

void drawConfirmedScreen() {
  display.clearDisplay();
  drawHeader("PUCK SET");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Location locked at:");

  display.setTextSize(2);
  display.setCursor(4, 32);
  display.print(approxDistanceM, 1);
  display.print(" m");

  display.setCursor(0, 56);
  display.setTextSize(1);
  display.print("returning...");

  display.display();
}

void drawCancelledScreen() {
  display.clearDisplay();
  drawHeader("CANCELLED");

  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("Puck placement");
  display.setCursor(0, 34);
  display.print("cancelled.");

  display.setCursor(0, 56);
  display.print("returning...");

  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_CONFIRM, INPUT_PULLUP);
  pinMode(BTN_CANCEL, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("Display not found!");
    while (1) { delay(1000); }
  }

  display.setRotation(DISPLAY_ROTATION);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("UI TEST BOOT");
  display.println("Set Puck screen");
  display.display();
  delay(600);

  lastEncState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  stateEnteredAt = millis();
}

void loop() {
  pollEncoder();

  // apply encoder movement (0.5 m per detent) only while actively setting
  if (uiState == SET_PUCK && encDelta != 0) {
    approxDistanceM += encDelta * 0.5f;
    if (approxDistanceM < 0) approxDistanceM = 0;
    if (approxDistanceM > 999) approxDistanceM = 999;
    headingDeg = (headingDeg + encDelta * 15) % 360;
    if (headingDeg < 0) headingDeg += 360;
    encDelta = 0;
  } else {
    encDelta = 0;
  }

  bool confirmPressed = buttonPressed(btnConfirm);
  bool cancelPressed  = buttonPressed(btnCancel);

  switch (uiState) {
    case SET_PUCK:
      if (confirmPressed) {
        uiState = CONFIRMED;
        stateEnteredAt = millis();
      } else if (cancelPressed) {
        uiState = CANCELLED;
        stateEnteredAt = millis();
      } else {
        drawSetPuckScreen();
      }
      break;

    case CONFIRMED:
      drawConfirmedScreen();
      if (millis() - stateEnteredAt > 1200) {
        uiState = SET_PUCK;
      }
      break;

    case CANCELLED:
      drawCancelledScreen();
      if (millis() - stateEnteredAt > 1200) {
        uiState = SET_PUCK;
      }
      break;
  }

  // redraw the transitional screens too (loop redraws state each pass)
  if (uiState == CONFIRMED) drawConfirmedScreen();
  if (uiState == CANCELLED) drawCancelledScreen();

  delay(15); // keep encoder poll responsive
}
