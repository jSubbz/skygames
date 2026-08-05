/*
    ESP32-C6 Pico Hardware Test
    ---------------------------
    Pinout:

    OLED SDA : GPIO22
    OLED SCL : GPIO23

    Potentiometer : GPIO0

    Button 1 : GPIO2
    Button 2 : GPIO3

    Active Buzzer : GPIO18

    RGB LED (Common Cathode)
      Red   : GPIO19
      Green : GPIO20
      Blue  : GPIO21

    Libraries:
      - Adafruit GFX
      - Adafruit SH110X
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------------
// Pin Definitions
// -------------------------

const uint8_t POT_PIN = 0;

const uint8_t BUTTON1_PIN = 2;
const uint8_t BUTTON2_PIN = 3;

const uint8_t BUZZER_PIN = 18;

const uint8_t LED_R = 19;
const uint8_t LED_G = 20;
const uint8_t LED_B = 21;

// -------------------------
// Helper Functions
// -------------------------

void setLED(bool r, bool g, bool b)
{
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

void beep(uint8_t times, uint16_t duration = 120)
{
    for (uint8_t i = 0; i < times; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);

        digitalWrite(BUZZER_PIN, LOW);
        delay(duration);
    }
}

void showCentered(const char *line1, const char *line2 = "")
{
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 8);
    display.println(line1);

    if (strlen(line2) > 0)
    {
        display.setCursor(0, 36);
        display.println(line2);
    }

    display.display();
}

// -------------------------
// Setup
// -------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    setLED(false, false, false);

    Wire.begin(22, 23);

    if (!display.begin(0x3C, true))
    {
        Serial.println("SH1106 not found!");

        while (true)
        {
            setLED(true, false, false);
            delay(200);
            setLED(false, false, false);
            delay(200);
        }
    }

    showCentered("ESP32", "TEST");
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-C6 HARDWARE SELF TEST");
    Serial.println("================================");

    // RGB Test

    Serial.println("Testing RGB LED");

    showCentered("LED", "RED");
    setLED(true, false, false);
    delay(700);

    showCentered("LED", "GREEN");
    setLED(false, true, false);
    delay(700);

    showCentered("LED", "BLUE");
    setLED(false, false, true);
    delay(700);

    showCentered("LED", "WHITE");
    setLED(true, true, true);
    delay(700);

    setLED(false, false, false);

    // Buzzer Test

    showCentered("BUZZER");
    Serial.println("Testing buzzer");

    beep(3);

    delay(500);

    showCentered("READY");
    delay(500);
}

// -------------------------
// Loop
// -------------------------

bool lastPressed = false;

void loop()
{
    int pot = analogRead(POT_PIN);

    bool btn1 = !digitalRead(BUTTON1_PIN);
    bool btn2 = !digitalRead(BUTTON2_PIN);

    // OLED

    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(0, 0);

    display.println("ESP32-C6 Hardware Test");
    display.println();

    display.print("Pot: ");
    display.println(pot);

    display.print("Button 1: ");
    display.println(btn1 ? "PRESSED" : "Released");

    display.print("Button 2: ");
    display.println(btn2 ? "PRESSED" : "Released");

    display.println();

    display.println("Rotate Pot");
    display.println("Press Buttons");

    display.display();

    // Serial

    Serial.print("Pot=");
    Serial.print(pot);

    Serial.print("  B1=");
    Serial.print(btn1);

    Serial.print("  B2=");
    Serial.println(btn2);

    // LED Status

    if (btn1 && btn2)
    {
        setLED(true, true, true);
    }
    else if (btn1)
    {
        setLED(true, false, false);
    }
    else if (btn2)
    {
        setLED(false, false, true);
    }
    else
    {
        setLED(false, true, false);
    }

    // Beep on new button press

    bool pressed = btn1 || btn2;

    if (pressed && !lastPressed)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(40);
        digitalWrite(BUZZER_PIN, LOW);
    }

    lastPressed = pressed;

    delay(50);
}