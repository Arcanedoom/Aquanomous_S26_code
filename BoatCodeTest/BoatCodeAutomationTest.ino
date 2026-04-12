#include <Wire.h>
#include <string>
#include <sstream>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"

// ================================================================
// SIMULATION SETTINGS
// ================================================================
#define TEST_MODE false

const uint8_t SIM_PACKETS[][2] = {
     {0,   0},   // stop, center
    {10,  10},   // 25% throttle, center
    {15,  15},   // 50% throttle, steer left
    {20,  20},   // 50% throttle, steer right
    {30,  30},
    {40,   40},   // stop, center
    {50,  50},   // 25% throttle, center
    {60,  60},   // 50% throttle, steer left
    {70,  70},   // 50% throttle, steer right
    {80,  80},   // 75% throttle, center
    {85, 85},   // full throttle, center
    {90,   90},   // 75% throttle, center
    {100,   100},
    {90,   90},   // stop, center
    {80,  80},   // 25% throttle, center
    {70,  70},   // 50% throttle, steer left
    {60,  60},   // 50% throttle, steer right
    {50,  50},   // 75% throttle, center
    {40, 40},   // full throttle, center
    {30,   30},
    {20,   20},   // stop, center
    {10,  10},   // 25% throttle, center
    {0,  50},   // 50% throttle, steer left
};
const int SIM_PACKET_COUNT = sizeof(SIM_PACKETS) / sizeof(SIM_PACKETS[0]);
const int SIM_HOLD_TIME_MS = 3000;
// ================================================================

// ================================================================
// THROTTLE RAMP SETTINGS
// ================================================================
#define THROTTLE_RAMP_MS  4000  // ms to ramp 0 -> 100% (increase = gentler)
#define THROTTLE_DECEL_MS 2000  // ms to ramp 100% -> 0
#define LOOP_INTERVAL_MS  100   // must match delay() at bottom of loop()
// ================================================================

// LoRa Constants
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 21
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 2
#define TIMEOUT 10
#define ALLOW_RECOVERY false

Air530ZClass GPS;

const uint8_t BOATMSG_CODE[8]   = "UK BOAT";
const uint8_t EMERGENCY_CODE[9] = "EMERGNCY";

union boatMsg {
    struct {
        uint8_t secretCode[8];
        float latitude;
        float longitude;
        float speed;
        bool emergencyFlag;
        uint8_t automationStatus;
    } status;
    uint8_t str[24];
};

static RadioEvents_t RadioEvents;

// Forward declarations
void controlThrottle();
void controlSteering();
void emergencyStopProcedure(const char* reason);
void sendEmergencySignal(const char* reason);
void sendStatusUpdate();
void updateDisplay();
void processUARTMessage(uint8_t *message, int length);
void executeAutomation();
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);

int OutofRangeNumber = 0;
int InRangeNumber = 0;
uint32_t lastHeartbeatTime = 0;
bool emergencyStop = false;

// uint8_t serialBuffer[BUFFER_SIZE];
uint8_t serialBuffer[256];
int bufferSize;

// Pin Definitions
#define DIRECTION_PIN          GPIO5   // LOW=forward, HIGH=reverse
#define STEERING_LEFT_PIN      GPIO7
#define STEERING_RIGHT_PIN     GPIO6
#define MOTOR_RELAY            GPIO4
#define KILL_SWITCH            GPIO9
#define STEERING_POTENTIOMETER ADC3
// PWM1 = GPIO3 (pin 10) — throttle signal to Curtis

int automationThrottle      = 0;
int automationSteering      = 50;
int currentSteeringPosition = 0;

int throttle      = 0;
int throttleState = 0;  // 1=forward, 0=stop

const int rudderMin       = 200;
const int rudderMax       = 4000;
const int rudderTolerance = 30;

static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10);

// ================================================================
// SIMULATION DISPLAY
// ================================================================

void simDisplay(int packetNum, uint8_t thr, uint8_t steer,
                int pot, int rawThrottle, uint32_t elapsed) {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "SIM MODE");
    display.setFont(ArialMT_Plain_10);
    char buf[40];
    snprintf(buf, sizeof(buf), "PKT %d/%d  T:%d%% S:%d%%",
             packetNum + 1, SIM_PACKET_COUNT, thr, steer);
    display.drawString(0, 20, buf);
    int actualPct = map(rawThrottle, 0, UINT16_MAX, 0, 100);
    snprintf(buf, sizeof(buf), "Motor: ~%d%%  Pot: %d", actualPct, pot);
    display.drawString(0, 32, buf);
    int targetPot = map(steer, 0, 100, rudderMin, rudderMax);
    snprintf(buf, sizeof(buf), "SteerTgt: %d Err: %d", targetPot, pot - targetPot);
    display.drawString(0, 44, buf);
    // Hold progress bar
    int bar = map(constrain((int)elapsed, 0, SIM_HOLD_TIME_MS), 0, SIM_HOLD_TIME_MS, 0, 120);
    display.drawRect(0, 55, 120, 7);
    display.fillRect(0, 55, bar, 7);
    display.display();
}

// ================================================================
// SIMULATION
// ================================================================

void runSimulation() {
    Serial.println("\n=== PACKET SIMULATION START ===");

    for (int i = 0; i < SIM_PACKET_COUNT; i++) {
        Serial.printf("\n[PKT %d/%d] Throttle: %d%%  Steering: %d%%\n",
                      i + 1, SIM_PACKET_COUNT,
                      SIM_PACKETS[i][0], SIM_PACKETS[i][1]);

        uint32_t holdStart = millis();
        while (millis() - holdStart < SIM_HOLD_TIME_MS) {

            // Re-send the same packet every loop cycle — mimics Pi sending at 10Hz
            uint8_t simPacket[2] = {SIM_PACKETS[i][0], SIM_PACKETS[i][1]};
            processUARTMessage(simPacket, BUFFER_SIZE);
            lastHeartbeatTime = millis();

            currentSteeringPosition = analogRead(STEERING_POTENTIOMETER);
            executeAutomation();

            int pct = map(throttle, 0, UINT16_MAX, 0, 100);
            simDisplay(i, SIM_PACKETS[i][0], SIM_PACKETS[i][1],
                       currentSteeringPosition, throttle, millis() - holdStart);

            Serial.printf("  Motor: ~%d%% (raw:%d)  Pot: %d  SteerTgt: %d\n",
                          pct, throttle, currentSteeringPosition,
                          map(automationSteering, 0, 100, rudderMin, rudderMax));
            delay(LOOP_INTERVAL_MS);
        }
    }

    // Safe stop — keep sending 0 throttle until fully decelerated
    Serial.println("\nDecelerating to stop...");
    while (throttle > 0) {
        uint8_t stopPacket[2] = {0, 50};
        processUARTMessage(stopPacket, BUFFER_SIZE);
        currentSteeringPosition = analogRead(STEERING_POTENTIOMETER);
        executeAutomation();
        int pct = map(throttle, 0, UINT16_MAX, 0, 100);
        Serial.printf("  Decelerating: ~%d%%\n", pct);
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, "STOPPING...");
        display.setFont(ArialMT_Plain_10);
        char buf[40];
        snprintf(buf, sizeof(buf), "Motor: ~%d%%", pct);
        display.drawString(0, 25, buf);
        display.display();
        delay(LOOP_INTERVAL_MS);
    }

    Serial.println("\n=== SIMULATION COMPLETE ===");
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "SIM DONE");
    display.setFont(ArialMT_Plain_10);
    char buf[40];
    snprintf(buf, sizeof(buf), "Final pot: %d", analogRead(STEERING_POTENTIOMETER));
    display.drawString(0, 25, buf);
    display.drawString(0, 40, "Re-upload to rerun");
    display.display();

    analogWrite(PWM1, 0);
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    while (true) { delay(1000); }
}
// ================================================================
// SETUP
// ================================================================

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);

    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.TxDone    = OnTxDone;
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

    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_RELAY, OUTPUT);
    pinMode(KILL_SWITCH, OUTPUT);
    pinMode(STEERING_POTENTIOMETER, INPUT);

    analogWrite(PWM1, 0);
    digitalWrite(DIRECTION_PIN, LOW);
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    digitalWrite(MOTOR_RELAY, HIGH);
    digitalWrite(KILL_SWITCH, HIGH);

    display.init();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, TEST_MODE ? "SIM MODE" : "AUTO MODE");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, "Starting in 3s...");
    display.display();

    delay(3000);
    lastHeartbeatTime = millis();
    
}

// ================================================================
// LOOP
// ================================================================

void loop() {
    if (TEST_MODE) {
        runSimulation();
        return;
    }

    //while (GPS.available()) { GPS.encode(GPS.read()); }

    bufferSize = Serial1.read(serialBuffer, TIMEOUT);
    if (bufferSize == BUFFER_SIZE) {
        processUARTMessage(serialBuffer, bufferSize);
        lastHeartbeatTime = millis();
    }
 //Serial.printf("Made it to heartbeat");
    if (millis() - lastHeartbeatTime > 10000) {
        emergencyStopProcedure("NO HEARTBEAT");
    }

    currentSteeringPosition = analogRead(STEERING_POTENTIOMETER);
    executeAutomation();
    updateDisplay();

    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 10000) {
        sendStatusUpdate();
        lastStatusTime = millis();
    }

//     Radio.Rx(1000);
//     delay(LOOP_INTERVAL_MS);
//     Radio.IrqProcess();
     }

// ================================================================
// CONTROL FUNCTIONS
// ================================================================

void executeAutomation() {
    if (emergencyStop) {
        emergencyStopProcedure("EMERGENCY STOP");
        return;
    }
    controlThrottle();
    controlSteering();
}

void controlThrottle() {
    int targetPWM  = (automationThrottle * UINT16_MAX) / 100;
    int rampStep   = (UINT16_MAX * LOOP_INTERVAL_MS) / THROTTLE_RAMP_MS;
    int decelStep  = (UINT16_MAX * LOOP_INTERVAL_MS) / THROTTLE_DECEL_MS;

    // DEBUG — remove once fixed
    Serial.printf("  [THROTTLE] cmd:%d%% state:%d raw:%d targetPWM:%d\n",
                  automationThrottle, throttleState, throttle, targetPWM);

    if (automationThrottle > 0) {
        if (throttleState != 1) {
            Serial.printf("  [THROTTLE] STATE RESET — was %d, now 1\n", throttleState);
            throttleState = 1;
        }
        throttle += rampStep;
        throttle = min(throttle, targetPWM);
        throttle = min(throttle, (14 * UINT16_MAX) / 20);
        digitalWrite(DIRECTION_PIN, LOW);    
        analogWrite(PWM1, throttle);
    } else {
        if (throttle > 0) {
            Serial.println("  [THROTTLE] DECELERATING");
            throttle -= decelStep;
            throttle = max(throttle, 0);
            analogWrite(PWM1, throttle);
        } else {
            if (throttleState != 0) {
                throttleState = 0;
                throttle      = 0;
                analogWrite(PWM1, 0);
            }
        }
    }
}

void controlSteering() {
    int targetPosition = map(automationSteering, 0, 100, rudderMin, rudderMax);
    int error          = currentSteeringPosition - targetPosition;

    // At target — stop
    if (abs(error) <= rudderTolerance) {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
        return;
    }

    // Drive toward target — non-blocking, returns immediately
    if (error > 0) {
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    } else {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
    }

    // Stuck detection
    static uint32_t turnStartTime = 0;
    static int lastPosition       = 0;

    if (currentSteeringPosition == lastPosition) {
        if (millis() - turnStartTime > 2000) {
            digitalWrite(STEERING_LEFT_PIN, LOW);
            digitalWrite(STEERING_RIGHT_PIN, LOW);
            emergencyStopProcedure("STEERING STUCK");
        }
    } else {
        turnStartTime = millis();
        lastPosition  = currentSteeringPosition;
    }
}

void processUARTMessage(uint8_t *message, int length) {
    if (length != BUFFER_SIZE) return;

    if (message[0] >= 100 || message[1] >= 100) {
        OutofRangeNumber++;
        InRangeNumber = 0;  // reset recovery progress on any bad packet
        Serial.printf("  [UART] Out of range! bad:%d  T:%d S:%d\n",
                      OutofRangeNumber, message[0], message[1]);
        if (OutofRangeNumber >= 3) {
            emergencyStopProcedure("OUT OF RANGE CMD");
        }
        return;
    }

    // Good packet
    OutofRangeNumber= 0;
    InRangeNumber++;
    Serial.printf("  [UART] Good packet, recovery:%d/3  T:%d S:%d\n",
                  InRangeNumber, message[0], message[1]);

    if ((emergencyStop) && (InRangeNumber >= 3)) {
        OutofRangeNumber = 0;
        if (ALLOW_RECOVERY) {
            emergencyStop = false;
            digitalWrite(KILL_SWITCH, HIGH);
            Serial.println("  [UART] RECOVERED — emergency stop cleared");
        } else {
            Serial.println("  [UART] Recovery disabled — staying in emergency stop");
        }
    }

    automationThrottle = message[0];
    automationSteering = message[1];
    Serial.printf("  [UART] T:%d S:%d\n", automationThrottle, automationSteering);
}
    

  

void emergencyStopProcedure(const char* reason) {
    if (emergencyStop) return;  // already in emergency stop, don't re-trigger
    emergencyStop = true;
    analogWrite(PWM1, 0);
    digitalWrite(STEERING_LEFT_PIN, LOW);
    digitalWrite(STEERING_RIGHT_PIN, LOW);
    digitalWrite(KILL_SWITCH, LOW);
    sendEmergencySignal(reason);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "EMERGENCY STOP");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 25, reason);
    display.display();
    Serial.printf("  [ESTOP] %s\n", reason);
    // no while(true) — loop() continues so UART can still be read for recovery
}

void updateDisplay() {
    int pot       = analogRead(STEERING_POTENTIOMETER);
    int targetPot = map(automationSteering, 0, 100, rudderMin, rudderMax);
    int actualPct = map(throttle, 0, UINT16_MAX, 0, 100);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "AUTO MODE");
    display.setFont(ArialMT_Plain_10);
    char buf[40];
    snprintf(buf, sizeof(buf), "Cmd:%d%% Motor:~%d%%",
             automationThrottle, actualPct);
    display.drawString(0, 20, buf);
    snprintf(buf, sizeof(buf), "Steer: %d%%  Pot: %d", automationSteering, pot);
    display.drawString(0, 32, buf);
    snprintf(buf, sizeof(buf), "Tgt: %d  Err: %d", targetPot, pot - targetPot);
    display.drawString(0, 44, buf);
    uint32_t hb = millis() - lastHeartbeatTime;
    snprintf(buf, sizeof(buf), "HB: %d.%ds", hb / 1000, (hb % 1000) / 100);
    display.drawString(0, 56, buf);
    display.display();
}

void sendStatusUpdate() {
    boatMsg msg;
    memcpy(msg.status.secretCode, BOATMSG_CODE, 8);
    if (GPS.location.isValid()) {
        msg.status.latitude  = (float)GPS.location.lat();
        msg.status.longitude = (float)GPS.location.lng();
        msg.status.speed     = (float)GPS.speed.kmph();
    } else {
        msg.status.latitude  = 0.0;
        msg.status.longitude = 0.0;
        msg.status.speed     = 0.0;
    }
    msg.status.emergencyFlag    = emergencyStop;
    msg.status.automationStatus = emergencyStop ? 2 : 1;
    Radio.Send(msg.str, sizeof(boatMsg));
}

void sendEmergencySignal(const char* reason) {
    boatMsg msg;
    memcpy(msg.status.secretCode, EMERGENCY_CODE, 8);
    msg.status.emergencyFlag    = true;
    msg.status.automationStatus = 2;
    for (int i = 0; i < 3; i++) {
        Radio.Send(msg.str, sizeof(boatMsg));
        delay(100);
    }
}

void OnTxDone() {}
void OnTxTimeout() {}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size == 8 && memcmp(payload, "STOPNOW!", 8) == 0) {
        emergencyStopProcedure("LORA STOP CMD");
    }
}
void OnRxTimeout() {}
