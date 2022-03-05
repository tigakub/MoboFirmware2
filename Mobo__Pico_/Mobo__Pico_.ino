#define VERSION "4.4.003b"

#define DEBUG false
#define LED_PIN 25

//* RF ****************************************************************************************************************
#include <SPI.h>
#include "RF24.h"

const uint64_t address[] = { 0x123456789ALL, 0xA987654321LL };

RF24 radio(6, 5); // (CE, CS)
bool radioNumber = 0;

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
#define MSG_THROTTLE_PERIOD 1000
unsigned long msgThrottleTime;

#define PING_PERIOD 250
unsigned long pingThrottleTime;
uint32_t pingCount = 0;

//* MOTOR ****************************************************************************************************************

#include "Mecanum.hpp"

#include "PicoPWM.hpp"

#define FL_DIR_PIN 12
#define FL_PWM_PIN 13
#define FL_ENC_PIN 14
#define FL_ALM_PIN 15

#define FR_DIR_PIN 8
#define FR_PWM_PIN 9
#define FR_ENC_PIN 10
#define FR_ALM_PIN 11

#define BL_DIR_PIN 16
#define BL_PWM_PIN 17
#define BL_ENC_PIN 18
#define BL_ALM_PIN 19

#define BR_DIR_PIN 20
#define BR_PWM_PIN 21
#define BR_ENC_PIN 22
#define BR_ALM_PIN 26

// Mobo's BLDC driver break lines are all connected to pin 27
#define BRK_PIN 27

// RP2040_PWM duty cycle ranges from 0.0 to 100.0.
#define MAX_DUTY_CYCLE 100.0

// Mobo 1's BLDCs have a max RPM of 70.0
#define MAX_RPM 70.0

// Motor poles: 4
// Pulses per revolution 3 * motor poles = 12
// Gear ratio 50:1
// Pulse to rpm factor = seconds per minute / pulses per revolution / gear ratio = 0.1
#define PULSE_TO_RPM 0.1

#define WHEEL_BASE_LENGTH 0.1905
#define WHEEL_BASE_WIDTH 0.1905
#define WHEEL_RADIUS 0.0635

PicoPWM
  frontLeftPWM(FL_PWM_PIN),
  frontRightPWM(FR_PWM_PIN),
  backLeftPWM(BL_PWM_PIN),
  backRightPWM(BR_PWM_PIN);

Mecanum mec(
  frontLeftPWM, frontRightPWM, backLeftPWM, backRightPWM,
  FL_DIR_PIN, FR_DIR_PIN, BL_DIR_PIN, BR_DIR_PIN,
  WHEEL_BASE_LENGTH, WHEEL_BASE_WIDTH, WHEEL_RADIUS);
  
float frontLeftRPM = 0.0;
float frontRightRPM = 0.0;
float backLeftRPM = 0.0;
float backRightRPM = 0.0;

/*
#define PWM_FREQ 2000

#define FR_DIR_PIN 8
#define FR_PWM_PIN 9
#define FR_ENC_PIN 10
#define FR_ALM_PIN 11

#define FL_DIR_PIN 12
#define FL_PWM_PIN 13
#define FL_ENC_PIN 14
#define FL_ALM_PIN 15

#define BL_DIR_PIN 16
#define BL_PWM_PIN 17
#define BL_ENC_PIN 18
#define BL_ALM_PIN 19

#define BR_DIR_PIN 20
#define BR_PWM_PIN 21
#define BR_ENC_PIN 22
#define BR_ALM_PIN 26

#define MOTOR_BRK_PIN 27

RP2040_PWM *frontLeftPWM = new RP2040_PWM(FL_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *frontRightPWM = new RP2040_PWM(FR_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *backLeftPWM = new RP2040_PWM(BL_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *backRightPWM = new RP2040_PWM(BR_PWM_PIN, PWM_FREQ, 0, false);

float frontLeftSpeed = 0.0;
float frontRightSpeed = 0.0;
float backLeftSpeed = 0.0;
float backRightSpeed = 0.0;

// Gear ratio 50:1
// Motor poles: 4
// Seconds per minute 60
// Pulses per revolution 3 * motor poles = 12
// Pulse to rpm factor = 60 / pulses per revolution / motor poles / gear ratio = 0.1
#define PULSE_TO_RPM_FACTOR 0.1
#define WHEEL_BASE_LENGTH 0.1905
#define WHEEL_BASE_WIDTH 0.1905
#define WHEEL_RADIUS 0.0635

// 2 PI * WHEEL_RADIUS * MAX_RPM / 60.0 (meters per second)
#define MAX_SPEED 0.4654793115

// 0.5 PI radians per second
#define MAX_ANGULAR_SPEED 1.5707963268

// 2 PI * 70.0 / 60.0 Radians per second
#define MAX_WHEEL_SPEED 7.3303828584

// Reciprocal of WHEEL_RADIUS
#define LINEAR_FACTOR 15.7480314961

// 0.5 WHEEL_BASE_LENGTH + 0.5 WHEEL_BASE_WIDTH
#define ANGULAR_FACTOR 0.1905

float frontLeftRPM = 0.0;
float frontRightRPM = 0.0;
float backLeftRPM = 0.0;
float backRightRPM = 0.0;

int breakState = HIGH;

float frontVel[2];
float backVel[2];
*/

//* IMU ****************************************************************************************************************
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

MbedI2C picoI2C(0, 1); // SDA, SCL

Adafruit_BNO055 bno(55, 0x28, &picoI2C);
#define IMU_THROTTLE_PERIOD 100
unsigned long imuThrottle;
sensors_event_t imuEvent;

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_COUNT 16
#define NEOPIXEL_PIN 28
Adafruit_NeoPixel neoPixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define NEOPIXEL_THROTTLE_PERIOD 100
unsigned long neoPixelThrottle;

//* NEOPIXEL ****************************************************************************************************************
#define NEOPIXEL_PULSE_DELAY 10
unsigned long neoPixelPulseTimer;
float neoPixelPulseValue = 0.0;
float neoPixelPulseBase = 0.0;
float neoPixelPulseStep = 1.0 / 64.0;
#define NEOPIXEL_PULSE_START 0.5
int remoteHeading;

//* KEEP_ALIVE ****************************************************************************************************************
#define KEEP_ALIVE_PERIOD 250
unsigned long keepAliveTime;

//* WATCHDOG ****************************************************************************************************************
#define WATCHDOG_PERIOD 4000
unsigned long watchdogTime;

#include <Watchdog.h>
#define watchdog (mbed::Watchdog::get_instance())
#define feed kick

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
  pingThrottleTime = millis();
}

//* MOTOR SETUP ********************************

#include "PicoPWM.hpp"

void motorSetup() {
  pinMode(FL_DIR_PIN, OUTPUT);
  pinMode(FR_DIR_PIN, OUTPUT);
  pinMode(BL_DIR_PIN, OUTPUT);
  pinMode(BR_DIR_PIN, OUTPUT);

  pinMode(MOTOR_BRK_PIN, OUTPUT);

  pinMode(FL_ENC_PIN, INPUT_PULLUP);
  pinMode(FR_ENC_PIN, INPUT_PULLUP);
  pinMode(BL_ENC_PIN, INPUT_PULLUP);
  pinMode(BR_ENC_PIN, INPUT_PULLUP);
  
  pinMode(FL_ALM_PIN, INPUT_PULLUP);
  pinMode(FR_ALM_PIN, INPUT_PULLUP);
  pinMode(BL_ALM_PIN, INPUT_PULLUP);
  pinMode(BR_ALM_PIN, INPUT_PULLUP);
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

//* NEOPIXEL SETUP ********************************
void neopixelSetup() {
  neoPixels.begin();
  neoPixelThrottle = millis();
  neoPixelPulseTimer = millis();
}

//* MULTICORE SETUP ********************************
#include <multicore.h>

uint32_t frontLeftPulseCount = 0;
uint32_t frontRightPulseCount = 0;
uint32_t backLeftPulseCount = 0;
uint32_t backRightPulseCount = 0;

// Forward declarations
void core1_setup();
void core1_loop();

uint32_t intercoreSignal;

void core1_init() {
  core1_setup();

  // Signal the first core that the second process has started
  multicore_fifo_push_blocking(0);
  
  // Retrieve the initial value from the first core
  intercoreSignal = multicore_fifo_pop_blocking();
  
  // Set up inter core synchronization
  multicore_lockout_victim_init();

  // Tight loop
  // It may be the Arduino tool chain is extra temperamental,
  // but the Pico seems to ignore any code in this loop unless
  // the code includes a function call. Perhaps it's some kind
  // of weird attempt at optimization?!? In other words, the
  // following did not work for me:
  /*
    while(1) counter++;
  */
  while(1) core1_loop();
}

void frontLeftEncoderIRQ() {
  frontLeftPulseCount++;
}

void frontRightEncoderIRQ() {
  frontRightPulseCount++;
}

void backLeftEncoderIRQ() {
  backLeftPulseCount++;
}

void backRightEncoderIRQ() {
  backRightPulseCount++;
}

unsigned long rpmSampleTime;

void multicoreSetup() {
  multicore_launch_core1(core1_init);

  multicore_fifo_pop_blocking();
  multicore_fifo_push_blocking(1);
  
  rpmSampleTime = millis();
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
  Serial.println("Mobo (Pico)");
  Serial.println(VERSION);
  #endif

  rfSetup();
  motorSetup();
  imuSetup();
  neopixelSetup();
  multicoreSetup();
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
  radio.flush_tx();
  uint8_t i = 0;
  if(!radio.write(static_cast<const void *>(&outgoing), static_cast<uint8_t>(sizeof(Msg)))) {
    radio.reUseTX();
  } else {
    result = true;
  }
  radio.startListening();
  return result;
}

#define INDEPENDENT_WHEEL_HEADING false
#define USE_IMU_FOR_ORIENTATION false

void handleIncoming()
{
  switch(incoming.msgId) {
    case TELEM:
      digitalWrite(LED_PIN, 1);
      watchdog.feed();
      delay(5);
      digitalWrite(LED_PIN, 0);
      Telem &t = incoming.payload.telem;

      remoteHeading = incoming.payload.telem.heading;

      #if 1
      float ljx = 0.0;
      if(t.ljx < 500) ljx = (static_cast<float>(t.ljx) - 500.0) / 500.0;
      if(t.ljx > 523) ljx = (static_cast<float>(t.ljx) - 523.0) / 500.0;

      float ljy = 0.0;
      
      if(t.ljy < 500) ljy = (static_cast<float>(t.ljy) - 500.0) / 500.0;
      if(t.ljy > 523) ljy = (static_cast<float>(t.ljy) - 523.0) / 500.0;
      
      float rjx = 2.0 * static_cast<float>(t.rjx) / 1023.0 - 1.0; rjx *= fabs(rjx);
      
      float forward = LINEAR_FACTOR * ljy * MAX_SPEED;
      float lateral = LINEAR_FACTOR * ljx * MAX_SPEED;
      float angular = LINEAR_FACTOR * ANGULAR_FACTOR * rjx * MAX_ANGULAR_SPEED;

      float flRadiansPerSec = forward - lateral - angular;
      float frRadiansPerSec = forward + lateral + angular;
      float blRadiansPerSec = forward + lateral - angular;
      float brRadiansPerSec = forward - lateral + angular;

      float maxRadiansPerSec = fabs(flRadiansPerSec);
      if(fabs(frRadiansPerSec) > maxRadiansPerSec) maxRadiansPerSec = fabs(frRadiansPerSec);
      if(fabs(blRadiansPerSec) > maxRadiansPerSec) maxRadiansPerSec = fabs(blRadiansPerSec);
      if(fabs(brRadiansPerSec) > maxRadiansPerSec) maxRadiansPerSec = fabs(brRadiansPerSec);

      if(maxRadiansPerSec > MAX_WHEEL_SPEED) {
        float normalizer = MAX_WHEEL_SPEED / maxRadiansPerSec;
        flRadiansPerSec *= normalizer;
        frRadiansPerSec *= normalizer;
        blRadiansPerSec *= normalizer;
        brRadiansPerSec *= normalizer;
      }

      float toDutyCycle = 100.0 / MAX_WHEEL_SPEED;
      
      frontLeftSpeed = flRadiansPerSec * toDutyCycle;
      frontRightSpeed = frRadiansPerSec * toDutyCycle;
      backLeftSpeed = blRadiansPerSec * toDutyCycle;
      backRightSpeed = brRadiansPerSec * toDutyCycle;
      
      #else
      // Dead zones for joysticks
      float front_ljx = 0.0;
      if(t.ljx < 500) front_ljx = (static_cast<float>(t.ljx) - 500.0) / 500.0;
      if(t.ljx > 523) front_ljx = (static_cast<float>(t.ljx) - 523.0) / 500.0;
      float back_ljx = front_ljx;

      float front_ljy = 0.0;
      if(t.ljy < 500) front_ljy = (static_cast<float>(t.ljy) - 500.0) / 500.0;
      if(t.ljy > 523) front_ljy = (static_cast<float>(t.ljy) - 523.0) / 500.0;
      float back_ljy = front_ljy;

      // Parabolic response obviates need for dead zone on the right stick
      float rjx = 2.0 * static_cast<float>(t.rjx) / 1023.0 - 1.0; rjx *= fabs(rjx);
      float rjy = 0.0;
      
      front_ljx += rjx;
      front_ljy += rjy;
      
      back_ljx -= rjx;
      back_ljy -= rjy;
      
      float front_ljm = sqrt(front_ljx * front_ljx + front_ljy * front_ljy);
      float back_ljm = sqrt(back_ljx * back_ljx + back_ljy * back_ljy);

      front_ljx /= front_ljm;
      front_ljy /= front_ljm;

      back_ljx /= back_ljm;
      back_ljy /= back_ljm;

      float frontHeading = atan2(front_ljx, front_ljy); // - headingDifferential;
      float backHeading = atan2(back_ljx, back_ljy); // - headingDifferential;
      
      #if DEBUG
      Serial.println("(" + String(frontHeading) + ", " + String(backHeading) + ")");
      #endif
      
      float frontLSpeed = cos(-0.25 * PI - frontHeading) * front_ljm;
      float frontRSpeed = cos(0.25 * PI - frontHeading) * front_ljm;
      float backLSpeed = cos(-0.25 * PI - backHeading) * back_ljm;
      float backRSpeed = cos(0.25 * PI - backHeading) * back_ljm;

      if(frontLSpeed > 1.0) frontLSpeed = 1.0;
      if(frontLSpeed < -1.0) frontLSpeed = -1.0;
      if(frontRSpeed > 1.0) frontRSpeed = 1.0;
      if(frontRSpeed < -1.0) frontRSpeed = -1.0;
      if(backLSpeed > 1.0) backLSpeed = 1.0;
      if(backLSpeed < -1.0) backLSpeed = -1.0;
      if(backRSpeed > 1.0) backRSpeed = 1.0;
      if(backRSpeed < -1.0) backRSpeed = -1.0;
      
      frontLeftSpeed = 100.0 * frontLSpeed;
      frontRightSpeed = 100.0 * frontRSpeed;
      backLeftSpeed = 100.0 * backLSpeed;
      backRightSpeed = 100.0 * backRSpeed;
      #endif
      
      keepAliveTime = millis();
      break;
  }
}

void calcRPM()
{
  float elapsed = static_cast<float>(currentTime - rpmSampleTime);

  float flPulseCount, frPulseCount, blPulseCount, brPulseCount;
  
  multicore_lockout_start_blocking();
  flPulseCount = static_cast<float>(frontLeftPulseCount);
  frPulseCount = static_cast<float>(frontRightPulseCount);
  blPulseCount = static_cast<float>(backLeftPulseCount);
  brPulseCount = static_cast<float>(backRightPulseCount);
  frontLeftPulseCount = 0;
  frontRightPulseCount = 0;
  backLeftPulseCount = 0;
  backRightPulseCount = 0;
  multicore_lockout_end_blocking();

  float countToHz = 1000.0 / elapsed;
  float flPulseFreq = countToHz * flPulseCount;
  float frPulseFreq = countToHz * frPulseCount;
  float blPulseFreq = countToHz * blPulseCount;
  float brPulseFreq = countToHz * brPulseCount;

  frontLeftRPM = flPulseFreq * PULSE_TO_RPM_FACTOR * ((frontLeftSpeed < 0.0) ? 1.0 : -1.0);
  frontRightRPM = frPulseFreq * PULSE_TO_RPM_FACTOR * ((frontRightSpeed > 0.0) ? 1.0 : -1.0);
  backLeftRPM = blPulseFreq * PULSE_TO_RPM_FACTOR * ((backLeftSpeed > 0.0) ? 1.0 : -1.0);
  backRightRPM = brPulseFreq * PULSE_TO_RPM_FACTOR * ((backRightSpeed < 0.0) ? 1.0 : -1.0);
  
  rpmSampleTime = currentTime;
}

void rfLoop() {
  if(radio.available()) {
    radio.read(static_cast<void *>(&incoming), sizeof(Msg));
    handleIncoming();
  }

  if((currentTime - msgThrottleTime) > MSG_THROTTLE_PERIOD) {
    msgThrottleTime = currentTime;
  }

  if((currentTime - pingThrottleTime) > PING_PERIOD) {
    calcRPM();
    outgoing.msgId = PING;
    outgoing.payload.ping.pingCount = pingCount;
    outgoing.payload.ping.heading = imuEvent.orientation.x;
    outgoing.payload.ping.frontLeftRPM = frontLeftRPM;
    outgoing.payload.ping.frontRightRPM = frontRightRPM;
    outgoing.payload.ping.backLeftRPM = backLeftRPM;
    outgoing.payload.ping.backRightRPM = backRightRPM;    
    pingCount++;
    if(!sendMsg()) {
      #if DEBUG
      Serial.println("Failed to send ping msg");
      #endif
    }
    pingThrottleTime = currentTime;
  }
}

//* MOTOR LOOP ****************************************************
void motorLoop() {
  breakState = ((currentTime - keepAliveTime) <= KEEP_ALIVE_PERIOD);
  digitalWrite(MOTOR_BRK_PIN, breakState);
  if(breakState) {
    frontLeftPWM->setPWM(FL_PWM_PIN, PWM_FREQ, abs(frontLeftSpeed));
    frontRightPWM->setPWM(FR_PWM_PIN, PWM_FREQ, abs(frontRightSpeed));
    backLeftPWM->setPWM(BL_PWM_PIN, PWM_FREQ, abs(backLeftSpeed));
    backRightPWM->setPWM(BR_PWM_PIN, PWM_FREQ, abs(backRightSpeed));
    
    digitalWrite(FL_DIR_PIN, frontLeftSpeed < 0.0);
    digitalWrite(FR_DIR_PIN, frontRightSpeed > 0.0);
    digitalWrite(BL_DIR_PIN, backLeftSpeed < 0.0);
    digitalWrite(BR_DIR_PIN, backRightSpeed > 0.0);
  } else {
    frontLeftPWM->setPWM(FL_PWM_PIN, PWM_FREQ, 0);
    frontRightPWM->setPWM(FR_PWM_PIN, PWM_FREQ, 0);
    backLeftPWM->setPWM(BL_PWM_PIN, PWM_FREQ, 0);
    backRightPWM->setPWM(BR_PWM_PIN, PWM_FREQ, 0);
  }
}

//* NEOPIXEL LOOP ****************************************************
float intensity(float x, float shift)
{
  return 1.0 - sqrt(cos(PI * (fmod(1.0 + x - shift, 1.0) - 0.5)));
}

void neopixelLoop() {
  if((currentTime - neoPixelPulseTimer) > NEOPIXEL_PULSE_DELAY) {
    neoPixelPulseTimer = currentTime;
    neoPixelPulseBase += neoPixelPulseStep;
    if(neoPixelPulseBase >= 1.0) {
      neoPixelPulseBase = 1.0;
      neoPixelPulseStep = -neoPixelPulseStep;
    }
    if(neoPixelPulseBase <= 0.0) {
      neoPixelPulseBase = 0.0;
      neoPixelPulseStep = fabs(neoPixelPulseStep);
    }
    neoPixelPulseValue = (NEOPIXEL_PULSE_START + (1.0 - NEOPIXEL_PULSE_START) * pow(neoPixelPulseBase, 5.0));
  }
  
  if((currentTime - neoPixelThrottle) > NEOPIXEL_THROTTLE_PERIOD) {
    neoPixelThrottle = currentTime;
    
    float normalized = (static_cast<float>(imuEvent.orientation.x) - remoteHeading) / 359.0;
    float r, g, b;
    for(int i = 0; i < 16; i++) {
      r = pow(intensity(static_cast<float>(i) / 15.0, normalized), 2.0) * neoPixelPulseValue;
      g = r * 31.0;
      b = (1.0 - r) * 63.0;
      r *= 255.0;
      neoPixels.setPixelColor(i, neoPixels.Color(r, g, b));
    }
    
    neoPixels.show();
  }
}

//* MAIN LOOP ****************************************************
void loop() {
  currentTime = millis();
  imuLoop();
  rfLoop();
  neopixelLoop();
  motorLoop();
}

void core1_setup() {
  attachInterrupt(digitalPinToInterrupt(FL_ENC_PIN), frontLeftEncoderIRQ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_PIN), frontRightEncoderIRQ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_PIN), backLeftEncoderIRQ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_PIN), backRightEncoderIRQ, CHANGE);
}

void core1_loop() {
}
