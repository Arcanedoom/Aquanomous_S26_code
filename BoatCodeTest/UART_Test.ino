#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "Arduino.h"

// ================================================================
// UART SETTINGS (RX2 TEST)
// ================================================================
#define RXD2 29   // Adjust if needed for your board
#define TXD2 30

#define BUFFER_SIZE 2

uint8_t serialBuffer[BUFFER_SIZE];

// ================================================================
// CONTROL VARIABLES
// ================================================================
int automationThrottle = 0;
int automationSteering = 50;

// ================================================================
// DISPLAY
// ================================================================
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

// ================================================================
// FUNCTION: Process UART Packet
// ================================================================
void processUARTMessage(uint8_t *message, int length) {
    if (length == BUFFER_SIZE) {
        automationThrottle = constrain(message[0], 0, 100);
        automationSteering = constrain(message[1], 0, 100);

        Serial.print("Parsed -> Throttle: ");
        Serial.print(automationThrottle);
        Serial.print("  Steering: ");
        Serial.println(automationSteering);
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);

    // IMPORTANT: Explicit RX2 definition
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);

    display.init();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "RX2 TEST MODE");

    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, "Waiting for data...");
    display.display();

    Serial.println("=== RX2 UART TEST STARTED ===");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    static int idx = 0;

    while (Serial1.available()) {
        uint8_t byteIn = Serial1.read();

        // Print raw byte
        Serial.print("RX BYTE: ");
        Serial.println(byteIn);

        // Fill buffer
        serialBuffer[idx++] = byteIn;

        // When full packet received
        if (idx == BUFFER_SIZE) {

            Serial.print("PACKET -> Throttle: ");
            Serial.print(serialBuffer[0]);
            Serial.print("  Steering: ");
            Serial.println(serialBuffer[1]);

            // Process packet
            processUARTMessage(serialBuffer, BUFFER_SIZE);

            // Reset buffer
            idx = 0;
        }
    }

    // Update display
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "RX2 TEST");

    display.setFont(ArialMT_Plain_10);

    char buf[40];
    snprintf(buf, sizeof(buf), "Throttle: %d%%", automationThrottle);
    display.drawString(0, 25, buf);

    snprintf(buf, sizeof(buf), "Steering: %d%%", automationSteering);
    display.drawString(0, 40, buf);

    display.display();

    delay(50);
}
