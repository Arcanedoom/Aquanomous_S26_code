#include "HT_SSD1306Wire.h"

// OLED
SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

// Pins
#define STEERING_LEFT_PIN   7
#define STEERING_RIGHT_PIN  6

// State
char steeringState = 'C';
unsigned long lastCmdTime = 0;
const unsigned long LINK_TIMEOUT = 2000; // ms

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  pinMode(STEERING_LEFT_PIN, OUTPUT);
  pinMode(STEERING_RIGHT_PIN, OUTPUT);

  digitalWrite(STEERING_LEFT_PIN, LOW);
  digitalWrite(STEERING_RIGHT_PIN, LOW);

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.clear();
  display.drawString(0, 0, "Steering Test");
  display.display();
}

void loop() {
  // UART RX
  if (Serial1.available()) {
    char cmd = Serial1.read();
    lastCmdTime = millis();

    switch (cmd) {
      case 'L':
        steeringState = 'L';
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
        break;

      case 'R':
        steeringState = 'R';
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
        break;

      case 'C':
        steeringState = 'C';
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
        break;
    }
  }

  // Link timeout safety
  if (millis() - lastCmdTime > LINK_TIMEOUT) {
    steeringState = 'C';
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
  }

  // OLED update
  display.clear();
  display.drawString(0, 0, "STEERING");

  if (steeringState == 'L') {
    display.drawString(0, 20, "L
