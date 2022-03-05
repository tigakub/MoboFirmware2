#define VERSION "4.4.001b"

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

#define MOTOR1_DIR_PIN 8
#define MOTOR1_PWM_PIN 9
#define MOTOR1_ENC_PIN 10
#define MOTOR1_ALM_PIN 11

#define MOTOR2_DIR_PIN 12
#define MOTOR2_PWM_PIN 13
#define MOTOR2_ENC_PIN 14
#define MOTOR2_ALM_PIN 15

#define MOTOR3_DIR_PIN 16
#define MOTOR3_PWM_PIN 17
#define MOTOR3_ENC_PIN 18
#define MOTOR3_ALM_PIN 19

#define MOTOR4_DIR_PIN 20
#define MOTOR4_PWM_PIN 21
#define MOTOR4_ENC_PIN 22
#define MOTOR4_ALM_PIN 26

#define MOTOR_BRK_PIN 27

RP2040_PWM *motor1pwm = new RP2040_PWM(MOTOR1_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *motor2pwm = new RP2040_PWM(MOTOR2_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *motor3pwm = new RP2040_PWM(MOTOR3_PWM_PIN, PWM_FREQ, 0, false);
RP2040_PWM *motor4pwm = new RP2040_PWM(MOTOR4_PWM_PIN, PWM_FREQ, 0, false);

float motor1Speed = 0.0;
float motor2Speed = 0.0;
float motor3Speed = 0.0;
float motor4Speed = 0.0;

// Gear ratio 50:1
// Motor poles: 4
// Seconds per minute 60
// Pulses per revolution 3 * motor poles = 12
// Pulse to rpm factor = 60 / pulses per revolution / motor poles / gear ratio = 0.1
#define PULSE_TO_RPM_FACTOR 0.1
#define WHEEL_RADIUS 0.076

float wheel1RPM = 0.0;
float wheel2RPM = 0.0;
float wheel3RPM = 0.0;
float wheel4RPM = 0.0;

int breakState = HIGH;

float frontVel[2];
float rearVel[2];
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
  Wheel::initStatics(
      BRK_PIN,
      MAX_DUTY_CYCLE,
      MAX_RPM,
      PULSE_TO_RPM
  );

  Wheel::releaseBreak();

  mec.start(33);
  /*
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);

  pinMode(MOTOR_BRK_PIN, OUTPUT);

  pinMode(MOTOR1_ALM_PIN, INPUT_PULLUP);
  pinMode(MOTOR2_ALM_PIN, INPUT_PULLUP);
  pinMode(MOTOR3_ALM_PIN, INPUT_PULLUP);
  pinMode(MOTOR4_ALM_PIN, INPUT_PULLUP);
  */
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

      // Dead zones for joysticks
      float ljx = 0.0;
      if(t.ljx < 500) ljx = (static_cast<float>(t.ljx) - 500.0) / 500.0;
      if(t.ljx > 523) ljx = (static_cast<float>(t.ljx) - 523.0) / 500.0;

      float ljy = 0.0;
      if(t.ljy < 500) ljy = (static_cast<float>(t.ljy) - 500.0) / 500.0;
      if(t.ljy > 523) ljy = (static_cast<float>(t.ljy) - 523.0) / 500.0;

      // Parabolic response obviates need for dead zone on the right stick
      float rjx = 2.0 * static_cast<float>(t.rjx) / 1023.0 - 1.0; rjx *= fabs(rjx);
      float rjy = 0.0;
      mec.set(ljy * MAX_RPM, ljx * MAX_RPM, rjx * 0.5 * PI);
      
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

  frontLeftRPM = flPulseFreq * PULSE_TO_RPM;
  frontRightRPM = frPulseFreq * PULSE_TO_RPM;
  backLeftPWM = blPulseFreq * PULSE_TO_RPM;
  backRightRPM = brPulseFreq * PULSE_TO_RPM;
  
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
  mec.yield(currentTime);
  /*
  breakState = ((currentTime - keepAliveTime) <= KEEP_ALIVE_PERIOD);
  digitalWrite(MOTOR_BRK_PIN, breakState);
  if(breakState) {
    motor1pwm->setPWM(MOTOR1_PWM_PIN, PWM_FREQ, abs(motor1Speed));
    motor2pwm->setPWM(MOTOR2_PWM_PIN, PWM_FREQ, abs(motor2Speed));
    motor3pwm->setPWM(MOTOR3_PWM_PIN, PWM_FREQ, abs(motor3Speed));
    motor4pwm->setPWM(MOTOR4_PWM_PIN, PWM_FREQ, abs(motor4Speed));
    
    digitalWrite(MOTOR1_DIR_PIN, motor1Speed < 0.0);
    digitalWrite(MOTOR2_DIR_PIN, motor2Speed > 0.0);
    digitalWrite(MOTOR3_DIR_PIN, motor3Speed > 0.0);
    digitalWrite(MOTOR4_DIR_PIN, motor4Speed < 0.0);
  } else {
    motor1pwm->setPWM(MOTOR1_PWM_PIN, PWM_FREQ, 0);
    motor2pwm->setPWM(MOTOR2_PWM_PIN, PWM_FREQ, 0);
    motor3pwm->setPWM(MOTOR3_PWM_PIN, PWM_FREQ, 0);
    motor4pwm->setPWM(MOTOR4_PWM_PIN, PWM_FREQ, 0);
  }
  */
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
  pinMode(FL_ENC_PIN, INPUT_PULLUP);
  pinMode(FR_ENC_PIN, INPUT_PULLUP);
  pinMode(BL_ENC_PIN, INPUT_PULLUP);
  pinMode(BR_ENC_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(FL_ENC_PIN), frontLeftEncoderIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_PIN), frontRightEncoderIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_PIN), backLeftEncoderIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_PIN), backRightEncoderIRQ, FALLING);
}

void core1_loop() {
}
