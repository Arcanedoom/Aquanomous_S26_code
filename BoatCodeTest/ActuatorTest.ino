#include "HT_SSD1306Wire.h"

// OLED
SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

// Test pins
#define PIN1 GPIO7
#define PIN2 GPIO6

bool state = false;

void setup() {
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);

  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.clear();
  display.display();
}

void loop() {

  // Toggle state
  state = !state;

  digitalWrite(PIN1, state);
  digitalWrite(PIN2, !state);

  // OLED display
  display.clear();
  display.drawString(0, 0, "GPIO TEST");

  display.drawString(0, 20, "PIN 7: ");
  display.drawString(80, 20, state ? "HIGH" : "LOW");

  display.drawString(0, 40, "PIN 6: ");
  display.drawString(80, 40, state ? "LOW" : "HIGH");

  display.display();

  delay(1000);
}
