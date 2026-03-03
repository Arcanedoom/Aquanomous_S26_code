#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "Arduino.h"

// Pins
#define THROTTLE_PIN  GPIO5
#define MOTOR_RELAY   GPIO4

// OLED
SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

void setup() {

  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(MOTOR_RELAY, OUTPUT);

  digitalWrite(THROTTLE_PIN, LOW);   // Forward direction
  digitalWrite(MOTOR_RELAY, HIGH);   // Enable motor driver

  display.init();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
}

void loop() {

  // Ramp Up
  for (int percent = 0; percent <= 100; percent += 5) {

    int pwmValue = map(percent, 0, 100, 0, UINT16_MAX);
    analogWrite(PWM1, pwmValue);

    updateDisplay(percent);
    delay(300);
  }

  delay(2000);

  // Ramp Down
  for (int percent = 100; percent >= 0; percent -= 5) {

    int pwmValue = map(percent, 0, 100, 0, UINT16_MAX);
    analogWrite(PWM1, pwmValue);

    updateDisplay(percent);
    delay(300);
  }

  delay(2000);
}

void updateDisplay(int percent) {

  display.clear();

  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "MOTOR TEST");

  display.setFont(ArialMT_Plain_10);

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "Throttle: %d%%", percent);
  display.drawString(0, 25, buffer);

  display.display();
}
