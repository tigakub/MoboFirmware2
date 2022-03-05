#define VERSION "4.4.003b"

#define DEBUG false

#define LED_PIN 17

//* RF ****************************************************************************************************************
#include <SPI.h>
#include "RF24.h"

const uint64_t address[] = { 0x123456789ALL, 0xA987654321LL };

RF24 radio(9, 8); // (CE, CS)
bool radioNumber = 1;

enum {
  PING = 1,
  TELEM = 2  
};

typedef struct Telem {
  int32_t ljx, ljy, rjx, rjy;
  int32_t heading;
  uint32_t bstates;
} Telem;

typedef struct Ping {
  uint32_t pingCount;
  int32_t heading;
  float frontLeftRPM;
  float frontRightRPM;
  float backLeftRPM;
  float backRightRPM;
} Ping;

typedef struct Msg {
  uint32_t msgId;
  union {
    Telem telem;
    Ping ping;
  } payload;
} Msg;

Msg incoming, outgoing;
#define MSG_THROTTLE_PERIOD 66
unsigned long msgThrottleTime;

//* IMU ****************************************************************************************************************
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno(55, 0x28);
#define IMU_THROTTLE_PERIOD 100
unsigned long imuThrottle;
sensors_event_t imuEvent;

//* WATCHDOG ****************************************************************************************************************
#include <avr/wdt.h>
#define WATCHDOG_PERIOD 5000
unsigned long watchdogTime;

class Watchdog
{
  public:
    Watchdog() { }
    void start(unsigned long iPeriod) {
      wdt_enable(WDTO_4S);
    }
    void stop() {
      wdt_disable();
    }
    void feed() {
      wdt_reset();
    }
};

Watchdog watchdog;

//* RF SETUP ********************************
void rfSetup() {
  while(!radio.begin()) {
    #if DEBUG
    Serial.println("RF failed to initialize nRF24L01");
    #endif
    delay(1000);
  }

  #if DEBUG
  Serial.println("nRF24L01 online");
  #endif

  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
  radio.enableAckPayload();
  radio.setRetries(0, 15);
  
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit
  radio.setPayloadSize(sizeof(Msg));

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);

  radio.startListening();
  
  msgThrottleTime = millis();
}

//* IMU SETUP ********************************
void imuSetup() {
  bool bnoReady = bno.begin();
  #if DEBUG
  if(!bnoReady) {
    Serial.println("BNO055 failure");
  } else {
    Serial.println("BNO055 online");
  }
  #endif

  adafruit_bno055_offsets_t bnoCalibData;
  bnoCalibData.accel_offset_x = 40;
  bnoCalibData.accel_offset_y = -47;
  bnoCalibData.accel_offset_z = -2;
  
  bnoCalibData.mag_offset_x = 211;
  bnoCalibData.mag_offset_y = 78;
  bnoCalibData.mag_offset_z = -316;
  
  bnoCalibData.gyro_offset_x = -2;
  bnoCalibData.gyro_offset_y = 1;
  bnoCalibData.gyro_offset_z = 0;
  
  bnoCalibData.accel_radius = 1000;
  bnoCalibData.mag_radius = 669;
  bno.setSensorOffsets(bnoCalibData);
  
  bno.setExtCrystalUse(true);

  imuThrottle = millis();
}

//* WATCHDOG SETUP ********************************
void watchdogSetup() {
  watchdog.stop();
  delay(WATCHDOG_PERIOD);
  watchdog.start(WATCHDOG_PERIOD);
  #if DEBUG
  Serial.println("Watchdog deployed");
  #endif
}

//* MAIN SETUP ****************************************************
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  #if DEBUG
  while(!Serial) {
    Serial.begin(115200);
    delay(1000);
  }
  #endif
  
  #if DEBUG
  Serial.println("Romo (Pro Micro)");
  Serial.println(VERSION);
  #endif

  rfSetup();
  imuSetup();
  watchdogSetup();
}

uint32_t currentTime;

//* IMU LOOP ****************************************************
void imuLoop() {
  if((currentTime - imuThrottle) > IMU_THROTTLE_PERIOD) {
    imuThrottle = currentTime;
    bno.getEvent(&imuEvent);
  }
}

//* RF LOOP ****************************************************
bool sendMsg()
{
  bool result = false;
  radio.stopListening();
  // radio.flush_tx(); // Automatically called by stopListening
  uint8_t i = 0;
  radio.writeFast(static_cast<const void *>(&outgoing), static_cast<uint8_t>(sizeof(Msg)));
  if(!radio.txStandBy(1000)) {
    // radio.reUseTX();
  } else {
    result = true;
  }
  radio.flush_tx();
  radio.startListening();
  return result;
}

bool ledToggle = 1;

void handleIncoming()
{
  switch(incoming.msgId) {
    case PING:
      digitalWrite(LED_PIN, ledToggle);
      watchdog.feed();
      ledToggle ^= 1;
      uint32_t pc = incoming.payload.ping.pingCount;
      #if DEBUG
      Serial.println("Received ping " + String(pc));
      Serial.println("wheelRPMs: "
        + String(incoming.payload.ping.frontLeftRPM) + ", "
        + String(incoming.payload.ping.frontRightRPM) + ", "
        + String(incoming.payload.ping.backLeftRPM) + ", "
        + String(incoming.payload.ping.backRightRPM));
        
      #endif
      break;
  }
}

void rfLoop() {
  if(radio.available()) {
    radio.read(static_cast<void *>(&incoming), sizeof(Msg));
    handleIncoming();
  }

  if((currentTime - msgThrottleTime) > MSG_THROTTLE_PERIOD) {
    outgoing.msgId = TELEM;
    // Joysticks are physically flipped for space
    outgoing.payload.telem.ljx = 1023 - analogRead(A0);
    outgoing.payload.telem.ljy = 1023 - analogRead(A1);
    outgoing.payload.telem.rjx = 1023 - analogRead(A2);
    outgoing.payload.telem.rjy = 1023 - analogRead(A3);
    outgoing.payload.telem.heading = imuEvent.orientation.x;
    if(!sendMsg()) {
      #if DEBUG
      Serial.println("Failed to send telem msg");
      #endif
    }
    msgThrottleTime = currentTime;
  }
}

//* MAIN LOOP ****************************************************
void loop() {
  currentTime = millis();
  imuLoop();
  rfLoop();
}
