#include <Wire.h>         
#include <string>
#include <sstream>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"


// LoRa Constants
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 21 // dBm  // 22 is the max value
#define LORA_BANDWIDTH 0 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 2 // Define the payload size here
#define TIMEOUT 5 // May need to adjust this

Air530ZClass GPS;

// Control code messages; this message will prevent bogus messages from giving incorrect signals to the boat
const uint8_t CONTROLLERMSG_CODE[8] = "UK RC  ";
const uint8_t BOATMSG_CODE[8] = "UK BOAT";

// This struct contains commands for the boat
// TODO: Change these to be ints to reflect new boat and controller code
union controllerMsg {
    struct {
      uint8_t secretCode[8];
      bool isGoingForward;
      bool isGoingBackward;
      bool isSteeringLeft;
      bool isSteeringRight;
      bool killswitch;
      bool automation;
    } status;
    uint8_t str[13];
  } inboundMsg;
  
union boatMsg {
    struct {
      uint8_t secretCode[8];
      int16_t lastRSSI;
      int16_t steeringPosition;
      float latitude;
      float longitude;
      float speed;
    } status;
    uint8_t str[24];
};

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
uint32_t signalTime = 0;
int16_t lastRSSI = 0;

// UART Communication Variables
uint8_t serialBuffer[BUFFER_SIZE];
int bufferSize;
bool automationMode = false; // Flag for automation mode

// Pin Definitions

#define THROTTLE_PIN GPIO5 //direction pin
#define STEERING_LEFT_PIN GPIO7
#define STEERING_RIGHT_PIN GPIO6
#define MOTOR_RELAY GPIO4 //
#define KILL_SWITCH GPIO9


// Controller and Automation Steering and Throttle Values
int controllerThrottle = 0;
int controllerSteering = 50; // 50 is centered
int automationThrottle = 0;
int automationSteering = 0;

// Legacy throttle and throttle state variables
int throttle = 0;
int throttleState = 0;
typedef void (*DisplayFunc)(void);
DisplayFunc outputs;

// Display Message
String serialMessage = "";
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr, freq, SDA, SCL, resolution, rst


void setup() {
    Serial1.begin(9600); // Initialize UART for communication with Raspberry Pi

    // LoRa Setup in controller this is a separate function so this might not be necessary?
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                        0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_RELAY, OUTPUT);
    pinMode(KILL_SWITCH, OUTPUT);
    digitalWrite(MOTOR_RELAY, HIGH);
    digitalWrite(KILL_SWITCH, HIGH);

    // Debug Displays
    outputs = displaySerial;
    // Initialize the display
    display.init();
    display.setFont(ArialMT_Plain_10);
}

void loop() {
    // Read and process UART messages from Raspberry Pi
    bufferSize = Serial1.read(serialBuffer, TIMEOUT);
    if (bufferSize) {
        processUARTMessage(serialBuffer, bufferSize);
    }

    // Debug Display
    // Clear the display
    display.clear();
    // Draw the current display method
    outputs();
    display.display();  

    Radio.Rx(500);
    delay(100);
    Radio.IrqProcess();

    // Perform actions based on automation or remote control
    doActions();

    delay(100);  // Reduce CPU usage?
    
    Radio.IrqProcess();
}

void doActions() {
  if (inboundMsg.status.killswitch) {
    killswitchLock();
  } 

  if ((millis() - signalTime) > 2000) {
    analogWrite(PWM1, 0);
    digitalWrite(GPIO6, LOW);
    digitalWrite(GPIO7, LOW);
    return;
  }
  automationMode = inboundMsg.status.automation;

  if (automationMode) {
      // Use automation values from Raspberry Pi
      automationControls(automationThrottle, automationSteering);
  } else {
      // Remote control values
      if (inboundMsg.status.isGoingForward) {
          if (throttleState != 1) {
            throttleState = 1;
            throttle = 1000;
          } else {
            throttle += 2000;
            throttle = min(throttle, (12*UINT16_MAX)/20);
          }
          digitalWrite(GPIO5, LOW);
          analogWrite(PWM1, throttle);
      } else if (inboundMsg.status.isGoingBackward){
          if (throttleState != -1) {
            throttleState = -1;
            throttle = 1000;
          } else {
            throttle += 2000;
            throttle = min(throttle, (7*UINT16_MAX)/20);
          }
          digitalWrite(GPIO5, HIGH);
          analogWrite(PWM1, throttle);
      } else {
          throttleState = 0;
          analogWrite(PWM1, 0);
      }

      if (inboundMsg.status.isSteeringLeft) {
          digitalWrite(STEERING_LEFT_PIN, HIGH);
          digitalWrite(STEERING_RIGHT_PIN, LOW);
      } else if (inboundMsg.status.isSteeringRight) {
          digitalWrite(STEERING_LEFT_PIN, LOW);
          digitalWrite(STEERING_RIGHT_PIN, HIGH);
      } else {
          digitalWrite(STEERING_LEFT_PIN, LOW);
          digitalWrite(STEERING_RIGHT_PIN, LOW);
      }
  }
}

// Potentiometer limits
int rudderMin = 200;
int rudderMax = 4000;
const int rudderTolerance = 30;

void automationControls(int throttleValue, int steeringValue) {
    // Throttle
    int targetPWM = (throttleValue * UINT16_MAX) / 100;

    if (throttleValue > 0) {
        if (throttleState != 1) {
            throttleState = 1;
            throttle = 1000;
        } else {
            throttle = min(throttle + 2000, targetPWM);
            throttle = min(throttle, (8 * UINT16_MAX) / 20);  // Optional cap
        }

        digitalWrite(GPIO5, LOW);  // Forward
        analogWrite(PWM1, throttle);
    } else {
        throttleState = 0;
        analogWrite(PWM1, 0);
    }

    // Steering
    int currentPos = analogRead(ADC3); 
    int targetPos = map(steeringValue, 0, 100, rudderMin, rudderMax);

    if (abs(currentPos - targetPos) <= rudderTolerance) {
        // Stop turning
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    } else if (currentPos > targetPos) {
        // Left?
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
    } else {
        // Right?
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    }
    // DEBUG
    if (steeringValue == 0) {
        // Dont turn if steering value is 0
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    }
}

// Figure out why this be making the throttle turn on high if its still doing it
void killswitchLock() {
  analogWrite(PWM1, 0);
  digitalWrite(STEERING_LEFT_PIN, LOW);
  digitalWrite(STEERING_RIGHT_PIN, LOW);
  digitalWrite(KILL_SWITCH, LOW);
  while(true) {
    delay(100);
  }
}

// Function to extract throttle and steering from received message
void processUARTMessage(uint8_t *message, int length) {
    if (length == 2) {  // Expect exactly 2 bytes (throttle and steering)
        automationThrottle = static_cast<int>(message[0]);
        automationSteering = static_cast<int>(message[1]);
    } else {
      // Do nothing
    }
}

//TX and RX functions
void OnTxDone() {
    Serial.println("Sent msg");
}
  
void OnTxTimeout() {
    Serial.println("Tx timeout");
}
  
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    signalTime = millis();
    lastRSSI = rssi;
    Serial.println("Received");
    if (size == sizeof(controllerMsg)) {
        controllerMsg temp;
        memcpy(temp.str, payload, sizeof(controllerMsg));
        if (memcmp(temp.status.secretCode, CONTROLLERMSG_CODE, sizeof(CONTROLLERMSG_CODE)) == 0) {
        memcpy(inboundMsg.str, payload, sizeof(controllerMsg));
        }

    }

}
  
void OnRxTimeout() {
    Serial.println("Rx timeout");
    killswitchLock();
}

//Serial Debug Display function
void displaySerial() {
    char messageBuffer[20];  // Small buffer for formatted output

    // Format: "T:XX S:XX" (Throttle and Steering values)
    snprintf(messageBuffer, sizeof(messageBuffer), "T:%d S:%d", serialBuffer[0], serialBuffer[1]);

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Automation Status");
    display.drawString(0, 20, messageBuffer);

}
