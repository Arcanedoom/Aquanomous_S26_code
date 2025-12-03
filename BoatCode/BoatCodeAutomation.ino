#include <Wire.h>
#include <string>
#include <sstream>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"

// LoRa Constants (for emergency killswitch/status reporting)
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 21 // dBm
#define LORA_BANDWIDTH 0 // [0: 125 kHz]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 2
#define TIMEOUT 5

Air530ZClass GPS;

// Emergency message codes
const uint8_t BOATMSG_CODE[8] = "UK BOAT";
const uint8_t EMERGENCY_CODE[8] = "EMERGNCY";

// Boat status struct for emergency reporting
union boatMsg {
    struct {
        uint8_t secretCode[8];
        float latitude;
        float longitude;
        float speed;
        bool emergencyFlag;
        uint8_t automationStatus; // 0=idle, 1=running, 2=error
    } status;
    uint8_t str[24];
};

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
uint32_t lastHeartbeatTime = 0;
bool emergencyStop = false;

// UART Communication Variables
uint8_t serialBuffer[BUFFER_SIZE];
int bufferSize;

// Pin Definitions
#define THROTTLE_PIN GPIO5
#define STEERING_LEFT_PIN GPIO7
#define STEERING_RIGHT_PIN GPIO6
#define MOTOR_RELAY GPIO4
#define KILL_SWITCH GPIO9
#define STEERING_POTENTIOMETER ADC3

// Automation control values
int automationThrottle = 0;  // 0-100%
int automationSteering = 50; // 0-100%, 50 = center
int currentSteeringPosition = 0;

// Steering parameters
const int rudderMin = 200;
const int rudderMax = 4000;
const int rudderTolerance = 30;
const int steeringCenter = 50;

// Display
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

void setup() {
    Serial1.begin(9600); // UART to Raspberry Pi
    
    // LoRa setup for emergency communications
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    // Pin setup
    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_RELAY, OUTPUT);
    pinMode(KILL_SWITCH, OUTPUT);
    pinMode(STEERING_POTENTIOMETER, INPUT);
    
    digitalWrite(MOTOR_RELAY, HIGH);
    digitalWrite(KILL_SWITCH, HIGH);
    digitalWrite(THROTTLE_PIN, LOW); // Default forward direction
    
    // Display setup
    display.init();
    display.setFont(ArialMT_Plain_10);
    
    // Initial display
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "AUTONOMOUS MODE");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, "Waiting for Pi...");
    display.display();
    
    lastHeartbeatTime = millis();
}

void loop() {
    // Read UART commands from Raspberry Pi
    bufferSize = Serial1.read(serialBuffer, TIMEOUT);
    if (bufferSize == BUFFER_SIZE) {
        processUARTMessage(serialBuffer, bufferSize);
        lastHeartbeatTime = millis(); // Reset heartbeat timer
    }
    
    // Check for heartbeat timeout (emergency stop if no communication)
    if (millis() - lastHeartbeatTime > 5000) { // 5 second timeout
        emergencyStopProcedure("NO HEARTBEAT");
    }
    
    // Read current steering position
    currentSteeringPosition = analogRead(STEERING_POTENTIOMETER);
    
    // Execute automation controls
    executeAutomation();
    
    // Update display
    updateDisplay();
    
    // Send status heartbeat via LoRa (every 10 seconds)
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 10000) {
        sendStatusUpdate();
        lastStatusTime = millis();
    }
    
    // Process LoRa events
    Radio.Rx(1000);
    delay(50);
    Radio.IrqProcess();
    
    delay(50); // Main loop delay
}

void executeAutomation() {
    if (emergencyStop) {
        emergencyStopProcedure("EMERGENCY STOP");
        return;
    }
    
    // Control throttle
    controlThrottle();
    
    // Control steering
    controlSteering();
}

void controlThrottle() {
    if (automationThrottle > 0) {
        // Map throttle percentage to PWM (0-100% -> 0-UINT16_MAX)
        int throttlePWM = map(automationThrottle, 0, 100, 0, UINT16_MAX);
        digitalWrite(THROTTLE_PIN, LOW); // Forward direction
        analogWrite(PWM1, throttlePWM);
    } else {
        analogWrite(PWM1, 0); // Stop motor
    }
}

void controlSteering() {
    // Map steering command to potentiometer range
    int targetPosition = map(automationSteering, 0, 100, rudderMin, rudderMax);
    
    // Check if we're close enough to target
    if (abs(currentSteeringPosition - targetPosition) <= rudderTolerance) {
        // Stop turning - we're at target
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
        return;
    }
    
    // Determine turning direction
    if (currentSteeringPosition > targetPosition) {
        // Turn left (or right depending on your setup)
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    } else {
        // Turn right (or left depending on your setup)
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
    }
    
    // Safety timeout - stop if stuck turning
    static uint32_t turnStartTime = 0;
    static int lastPosition = 0;
    
    if (currentSteeringPosition == lastPosition) {
        if (millis() - turnStartTime > 2000) { // 2 second timeout
            digitalWrite(STEERING_LEFT_PIN, LOW);
            digitalWrite(STEERING_RIGHT_PIN, LOW);
            emergencyStopProcedure("STEERING STUCK");
        }
    } else {
        turnStartTime = millis();
        lastPosition = currentSteeringPosition;
    }
}

void processUARTMessage(uint8_t *message, int length) {
    if (length == BUFFER_SIZE) {
        automationThrottle = constrain(message[0], 0, 100);
        automationSteering = constrain(message[1], 0, 100);
        
        // Check for emergency stop command (throttle=255)
        if (message[0] == 255) {
            emergencyStopProcedure("PI COMMAND");
        }
    }
}

void emergencyStopProcedure(const char* reason) {
    emergencyStop = true;
    
    // Stop all motors
    analogWrite(PWM1, 0);
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    digitalWrite(KILL_SWITCH, LOW);
    
    // Send emergency LoRa message
    sendEmergencySignal(reason);
    
    // Update display
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "EMERGENCY STOP");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, reason);
    display.display();
    
    // Halt execution
    while(true) {
        delay(1000);
        // Flash kill switch LED if available
        digitalWrite(KILL_SWITCH, !digitalRead(KILL_SWITCH));
    }
}

void updateDisplay() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "AUTO MODE");
    
    display.setFont(ArialMT_Plain_10);
    
    // Throttle and steering status
    char statusBuffer[40];
    snprintf(statusBuffer, sizeof(statusBuffer), "Throttle: %d%%", automationThrottle);
    display.drawString(0, 20, statusBuffer);
    
    snprintf(statusBuffer, sizeof(statusBuffer), "Steering: %d/%d", automationSteering, currentSteeringPosition);
    display.drawString(0, 35, statusBuffer);
    
    // Heartbeat status
    uint32_t timeSinceHeartbeat = millis() - lastHeartbeatTime;
    snprintf(statusBuffer, sizeof(statusBuffer), "HB: %d.%ds", timeSinceHeartbeat / 1000, (timeSinceHeartbeat % 1000) / 100);
    display.drawString(0, 50, statusBuffer);
    
    display.display();
}

void sendStatusUpdate() {
    boatMsg msg;
    memcpy(msg.status.secretCode, BOATMSG_CODE, 8);
    
    // Get GPS data if available
    if (GPS.getHour() != 0) { // Check if GPS has fix
        msg.status.latitude = GPS.getLatitude();
        msg.status.longitude = GPS.getLongitude();
        msg.status.speed = GPS.getSpeed();
    } else {
        msg.status.latitude = 0.0;
        msg.status.longitude = 0.0;
        msg.status.speed = 0.0;
    }
    
    msg.status.emergencyFlag = emergencyStop;
    msg.status.automationStatus = emergencyStop ? 2 : 1;
    
    Radio.Send(msg.str, sizeof(boatMsg));
}

void sendEmergencySignal(const char* reason) {
    boatMsg msg;
    memcpy(msg.status.secretCode, EMERGENCY_CODE, 8);
    msg.status.emergencyFlag = true;
    msg.status.automationStatus = 2;
    
    // Try to send multiple times
    for (int i = 0; i < 3; i++) {
        Radio.Send(msg.str, sizeof(boatMsg));
        delay(100);
    }
}

// LoRa event handlers
void OnTxDone() {
    // Optional: Add confirmation logging
}

void OnTxTimeout() {
    // Optional: Handle timeout
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Only listen for emergency stop commands
    if (size == 8) {
        if (memcmp(payload, "STOPNOW!", 8) == 0) {
            emergencyStopProcedure("LORA STOP CMD");
        }
    }
}

void OnRxTimeout() {
    // Normal for Rx timeout in our usage
}
