#define VERSION "4.2.003b"

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

#define PING_PERIOD 500
unsigned long pingThrottleTime;
uint32_t pingCount = 0;

//* MOTOR ****************************************************************************************************************
#include <RP2040_PWM.h>

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

int breakState = HIGH;

float frontVel[2];
float rearVel[2];

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
void motorSetup() {
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);

  pinMode(MOTOR_BRK_PIN, OUTPUT);

  pinMode(MOTOR1_ENC_PIN, INPUT);
  pinMode(MOTOR2_ENC_PIN, INPUT);
  pinMode(MOTOR3_ENC_PIN, INPUT);
  pinMode(MOTOR4_ENC_PIN, INPUT);
  
  pinMode(MOTOR1_ALM_PIN, INPUT);
  pinMode(MOTOR2_ALM_PIN, INPUT);
  pinMode(MOTOR3_ALM_PIN, INPUT);
  pinMode(MOTOR4_ALM_PIN, INPUT);
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

#define USE_IMU_FOR_ORIENTATION false

void handleIncoming()
{
  switch(incoming.msgId) {
    case TELEM:
      remoteHeading = incoming.payload.telem.heading;
      
      #if USE_IMU_FOR_ORIENTATION
      float normalizedLocalHeading = static_cast<float>(imuEvent.orientation.x) / 359.0;
      float normalizedRemoteHeading = static_cast<float>((remoteHeading + 540) % 360) / 359.0;
      if(normalizedLocalHeading > 0.5) normalizedLocalHeading = 0.5 - normalizedLocalHeading;
      if(normalizedRemoteHeading > 0.5) normalizedRemoteHeading = 0.5 - normalizedRemoteHeading;
      float headingDifferential = normalizedLocalHeading - normalizedRemoteHeading;
      #endif
      
      digitalWrite(LED_PIN, 1);
      watchdog.feed();
      delay(5);
      digitalWrite(LED_PIN, 0);
      Telem &t = incoming.payload.telem;

      // Dead zones for joysticks
      float front_ljx = 0.0;
      if(t.ljx < 500) front_ljx = (static_cast<float>(t.ljx) - 500.0) / 500.0;
      if(t.ljx > 523) front_ljx = (static_cast<float>(t.ljx) - 523.0) / 500.0;
      float rear_ljx = front_ljx;

      float front_ljy = 0.0;
      if(t.ljy < 500) front_ljy = (static_cast<float>(t.ljy) - 500.0) / 500.0;
      if(t.ljy > 523) front_ljy = (static_cast<float>(t.ljy) - 523.0) / 500.0;
      float rear_ljy = front_ljy;

      #if USE_IMU_FOR_ORIENTATION
      float rjx = 100.0 * headingDifferential * fabs(headingDifferential);
      #else
      // Parabolic response obviates need for dead zone on the right stick
      /*
      float rjx = 0.0;
      if(t.rjx < 500) rjx = -pow((static_cast<float>(t.rjx) - 500.0) / 500.0, 2.0);
      if(t.rjx > 523) rjx = pow((static_cast<float>(t.rjx) - 523.0) / 500.0, 2.0);
      */
      float rjx = 2.0 * static_cast<float>(t.rjx) / 1023.0 - 1.0; rjx *= fabs(rjx);
      #endif
      float rjy = 0.0;
      
      front_ljx += rjx;
      front_ljy += rjy;
      
      rear_ljx -= rjx;
      rear_ljy -= rjy;
      
      float front_ljm = sqrt(front_ljx * front_ljx + front_ljy * front_ljy);
      float rear_ljm = sqrt(rear_ljx * rear_ljx + rear_ljy * rear_ljy);

      front_ljx /= front_ljm;
      front_ljy /= front_ljm;

      rear_ljx /= rear_ljm;
      rear_ljy /= rear_ljm;

      float frontHeading = atan2(front_ljx, front_ljy); // - headingDifferential;
      float rearHeading = atan2(rear_ljx, rear_ljy); // - headingDifferential;
      
      #if DEBUG
      Serial.println("(" + String(frontHeading) + ", " + String(rearHeading) + ")");
      #endif
      
      float frontLSpeed = cos(0.25 * PI - frontHeading) * front_ljm;
      float frontRSpeed = cos(-0.25 * PI - frontHeading) * front_ljm;
      float rearLSpeed = cos(-0.25 * PI - rearHeading) * rear_ljm;
      float rearRSpeed = cos(0.25 * PI - rearHeading) * rear_ljm;

      if(frontLSpeed > 1.0) frontLSpeed = 1.0;
      if(frontLSpeed < -1.0) frontLSpeed = -1.0;
      if(frontRSpeed > 1.0) frontRSpeed = 1.0;
      if(frontRSpeed < -1.0) frontRSpeed = -1.0;
      if(rearLSpeed > 1.0) rearLSpeed = 1.0;
      if(rearLSpeed < -1.0) rearLSpeed = -1.0;
      if(rearRSpeed > 1.0) rearRSpeed = 1.0;
      if(rearRSpeed < -1.0) rearRSpeed = -1.0;
      
      motor1Speed = 100.0 * frontLSpeed;
      motor2Speed = 100.0 * frontRSpeed;
      motor3Speed = 100.0 * rearRSpeed;
      motor4Speed = 100.0 * rearLSpeed;
      
      keepAliveTime = millis();
      break;
  }
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
    outgoing.msgId = PING;
    outgoing.payload.ping.pingCount = pingCount;
    outgoing.payload.ping.heading = imuEvent.orientation.x;
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
    motor1pwm->setPWM(MOTOR1_PWM_PIN, PWM_FREQ, abs(motor1Speed));
    motor2pwm->setPWM(MOTOR2_PWM_PIN, PWM_FREQ, abs(motor2Speed));
    motor3pwm->setPWM(MOTOR3_PWM_PIN, PWM_FREQ, abs(motor3Speed));
    motor4pwm->setPWM(MOTOR4_PWM_PIN, PWM_FREQ, abs(motor4Speed));
    
    digitalWrite(MOTOR1_DIR_PIN, motor1Speed > 0.0);
    digitalWrite(MOTOR2_DIR_PIN, motor2Speed < 0.0);
    digitalWrite(MOTOR3_DIR_PIN, motor3Speed < 0.0);
    digitalWrite(MOTOR4_DIR_PIN, motor4Speed > 0.0);
  } else {
    motor1pwm->setPWM(MOTOR1_PWM_PIN, PWM_FREQ, 0);
    motor2pwm->setPWM(MOTOR2_PWM_PIN, PWM_FREQ, 0);
    motor3pwm->setPWM(MOTOR3_PWM_PIN, PWM_FREQ, 0);
    motor4pwm->setPWM(MOTOR4_PWM_PIN, PWM_FREQ, 0);
    
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
