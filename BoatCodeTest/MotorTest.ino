#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// Pin Definitions (CubeCell AB02S)
#define THROTTLE_PIN        GPIO5
#define STEERING_LEFT_PIN   GPIO7
#define STEERING_RIGHT_PIN  GPIO6
#define MOTOR_RELAY         GPIO4
#define KILL_SWITCH         GPIO9
#define STEERING_POT        ADC3

// RC Filter note:
// SEPEX expects 0-5V analog. CubeCell outputs 3.3V PWM.
// You need: GPIO5 -> 10kOhm -> SEPEX input, with 10uF cap to GND
// AND a 3.3V->5V level shifter (e.g. MCP6001 op-amp) on the output.
// Without this, throttle will be unreliable and max at ~66% of range.
// CubeCell analogWrite() is 0-255 (8-bit), ~490Hz by default.

// Test Parameters (adjust these to tune your test)
const int TEST_THROTTLE_STEP = 5;    // How fast to ramp (0-100%)
const int TEST_THROTTLE_HOLD = 50;   // % throttle to hold during steering test
const int STEERING_MOVE_MS   = 800;  // How long to pulse each steering direction
const int STEP_DELAY_MS      = 200;  // Delay between throttle steps

// Display
SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

void setThrottle(int percent) {
    percent = constrain(percent, 0, 100);
    int duty = map(percent, 0, 100, 0, UINT16_MAX);
    analogWrite(THROTTLE_PIN, duty);
}

void steerLeft(int ms) {
    digitalWrite(STEERING_LEFT_PIN, HIGH);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    delay(ms);
    digitalWrite(STEERING_LEFT_PIN, LOW);
}

void steerRight(int ms) {
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, HIGH);
    delay(ms);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
}

void steerStop() {
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
}

void showDisplay(const char* title, int throttlePct, int potReading) {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, title);
    display.setFont(ArialMT_Plain_10);
    char buf[40];
    snprintf(buf, sizeof(buf), "Throttle: %d%%", throttlePct);
    display.drawString(0, 22, buf);
    snprintf(buf, sizeof(buf), "Pot: %d", potReading);
    display.drawString(0, 35, buf);
    display.drawString(0, 50, "Relay: ON  Kill: ON");
    display.display();
}

void setup() {
    Serial.begin(115200);

    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_RELAY, OUTPUT);
    pinMode(KILL_SWITCH, OUTPUT);
    pinMode(STEERING_POT, INPUT);

    setThrottle(0);
    steerStop();
    digitalWrite(MOTOR_RELAY, HIGH);
    digitalWrite(KILL_SWITCH, HIGH);

    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "MOTOR TEST");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, "Starting in 3s...");
    display.display();

    Serial.println("=== MOTOR + STEERING TEST ===");
    Serial.println("Relay ON, Kill ON. Starting in 3s.");
    delay(3000);
}

void loop() {
    // PHASE 1: Throttle ramp up
    Serial.println("\n[PHASE 1] Throttle ramp UP (0 -> 100%)");
    for (int t = 0; t <= 100; t += TEST_THROTTLE_STEP) {
        setThrottle(t);
        int pot = analogRead(STEERING_POT);
        showDisplay("RAMP UP", t, pot);
        Serial.printf("  Throttle: %3d%%  Pot: %4d\n", t, pot);
        delay(STEP_DELAY_MS);
    }

    Serial.println("[HOLD] Full throttle 2s");
    showDisplay("FULL THRTL", 100, analogRead(STEERING_POT));
    delay(2000);

    // PHASE 2: Throttle ramp down
    Serial.println("\n[PHASE 2] Throttle ramp DOWN (100 -> 0%)");
    for (int t = 100; t >= 0; t -= TEST_THROTTLE_STEP) {
        setThrottle(t);
        int pot = analogRead(STEERING_POT);
        showDisplay("RAMP DOWN", t, pot);
        Serial.printf("  Throttle: %3d%%  Pot: %4d\n", t, pot);
        delay(STEP_DELAY_MS);
    }

    setThrottle(0);
    Serial.println("[STOP] Throttle stopped. Pausing 2s before steering test.");
    showDisplay("STOP", 0, analogRead(STEERING_POT));
    delay(2000);

    // PHASE 3: Steering test (motor stopped for safety)
    Serial.println("\n[PHASE 3] Steering LEFT");
    showDisplay("STEER LEFT", 0, analogRead(STEERING_POT));
    steerLeft(STEERING_MOVE_MS);
    Serial.printf("  Pot after left: %d\n", analogRead(STEERING_POT));
    delay(500);

    Serial.println("[PHASE 3] Steering RIGHT");
    showDisplay("STEER RIGHT", 0, analogRead(STEERING_POT));
    steerRight(STEERING_MOVE_MS);
    Serial.printf("  Pot after right: %d\n", analogRead(STEERING_POT));
    delay(500);

    Serial.println("[PHASE 3] Returning to CENTER");
    showDisplay("STEER CTR", 0, analogRead(STEERING_POT));
    steerRight(STEERING_MOVE_MS / 2);
    steerStop();
    delay(500);

    // Done
    Serial.println("\n[DONE] Test complete. Halting.");
    Serial.printf("Final pot reading: %d\n", analogRead(STEERING_POT));
    Serial.println("Expected pot range: 200 (full left) to 4000 (full right)");
    showDisplay("TEST DONE", 0, analogRead(STEERING_POT));
    setThrottle(0);
    steerStop();

    while (true) { delay(1000); }
}
