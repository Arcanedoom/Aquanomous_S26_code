#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"

#define TEST_KILL_SWITCH 1   // ← SET TO 0 FOR NORMAL OPERATION

// ================= LoRa Constants =================
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 21
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// ================= Pin Definitions =================
#define THROTTLE_PIN GPIO5
#define STEERING_LEFT_PIN GPIO7
#define STEERING_RIGHT_PIN GPIO6
#define MOTOR_RELAY GPIO4
#define KILL_SWITCH GPIO9
#define STEERING_POTENTIOMETER ADC3

// ================= Globals =================
Air530ZClass GPS;
static RadioEvents_t RadioEvents;

int automationThrottle = 0;
int automationSteering = 50;
int currentSteeringPosition = 0;

bool emergencyStop = false;

uint32_t lastHeartbeatTime = 0;

// ================= Steering Parameters =================
const int rudderMin = 200;
const int rudderMax = 4000;
const int rudderTolerance = 30;

// ================= OLED =================
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

// ================= TEST PACKETS =================
#if TEST_KILL_SWITCH
const uint8_t testCommands[][2] = {
    {50, 50},
    {100, 0},
    {0, 100},
    {150, 50},   // invalid
    {50, 150},   // invalid
    {255, 255},  // invalid
    {200, 200}   // invalid
};

const int NUM_TESTS = sizeof(testCommands) / 2;
int currentTestIndex = 0;
uint32_t lastTestTime = 0;
#endif

// ================= Function Prototypes =================
void processUARTMessage(uint8_t *message, int length);
void executeAutomation();
void controlThrottle();
void controlSteering();
void emergencyStopProcedure(const char* reason);
void updateDisplay();

// ================= SETUP =================
void setup() {

    Serial.begin(115200);
    Serial1.begin(9600);

    // PWM setup for ESP32
    ledcSetup(0, 20000, 16);
    ledcAttachPin(THROTTLE_PIN, 0);

    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_RELAY, OUTPUT);
    pinMode(KILL_SWITCH, OUTPUT);
    pinMode(STEERING_POTENTIOMETER, INPUT);

    digitalWrite(MOTOR_RELAY, HIGH);
    digitalWrite(KILL_SWITCH, HIGH);

    // OLED
    display.init();
    display.setFont(ArialMT_Plain_10);

    // LoRa setup
    RadioEvents.TxDone = NULL;
    RadioEvents.TxTimeout = NULL;
    RadioEvents.RxDone = NULL;
    RadioEvents.RxTimeout = NULL;

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

    lastHeartbeatTime = millis();
}

// ================= LOOP =================
void loop() {

#if TEST_KILL_SWITCH
    if (millis() - lastTestTime > 4000) {

        uint8_t packet[2];
        packet[0] = testCommands[currentTestIndex][0];
        packet[1] = testCommands[currentTestIndex][1];

        Serial.print("TEST SEND: ");
        Serial.print(packet[0]);
        Serial.print(", ");
        Serial.println(packet[1]);

        Radio.Send(packet, 2);

        currentTestIndex++;
        if (currentTestIndex >= NUM_TESTS) {
            currentTestIndex = 0;
        }

        lastTestTime = millis();
    }
#endif

    currentSteeringPosition = analogRead(STEERING_POTENTIOMETER);

    if (!emergencyStop) {
        executeAutomation();
    }

    updateDisplay();

    Radio.Rx(1000);
    Radio.IrqProcess();

    delay(50);
}

// ================= COMMAND PROCESSING =================
void processUARTMessage(uint8_t *message, int length) {

    if (length != 2) return;

    uint8_t rawThrottle = message[0];
    uint8_t rawSteering = message[1];

    static int invalidCount = 0;

    if (rawThrottle > 100 || rawSteering > 100) {

        invalidCount++;

        if (invalidCount >= 1) {
            emergencyStopProcedure("INVALID CMD");
        }

        return;
    }

    invalidCount = 0;

    automationThrottle = rawThrottle;
    automationSteering = rawSteering;
}

// ================= AUTOMATION =================
void executeAutomation() {
    controlThrottle();
    controlSteering();
}

void controlThrottle() {
    int throttlePWM = map(automationThrottle, 0, 100, 0, 65535);
    ledcWrite(0, throttlePWM);
}

void controlSteering() {

    int targetPosition = map(automationSteering, 0, 100, rudderMin, rudderMax);

    if (abs(currentSteeringPosition - targetPosition) <= rudderTolerance) {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
        return;
    }

    if (currentSteeringPosition > targetPosition) {
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    } else {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
    }
}

// ================= EMERGENCY =================
void emergencyStopProcedure(const char* reason) {

    emergencyStop = true;

    ledcWrite(0, 0);
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    digitalWrite(KILL_SWITCH, LOW);

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "EMERGENCY STOP");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, reason);
    display.display();

#if TEST_KILL_SWITCH
    delay(3000);
    ESP.restart();  // allow repeated testing
#else
    while (true) { delay(1000); }
#endif
}

// ================= DISPLAY =================
void updateDisplay() {

    display.clear();

    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "AUTO MODE");

    display.setFont(ArialMT_Plain_10);

    display.drawString(0, 20, "Throttle: " + String(automationThrottle));
    display.drawString(0, 35, "Steering: " + String(automationSteering));

    display.display();
}
