#define VERSION "4.0.004b"

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

//* WATCHDOG ****************************************************************************************************************

#define WATCHDOG_PERIOD 4000
unsigned long watchdogTime;

#include <Watchdog.h>
#define watchdog (mbed::Watchdog::get_instance())
#define feed kick

/*
class RadioStream : public Stream
{
  public:
    RadioStream(RF24 &iRadio)
    : Stream(), radio(iRadio) { }

    virtual int available() {
      return radio.available();
    }

    virtual int read() {
      uint8_t buf;
      byte pipeNo;
      if(radio.available(&pipeNo)) {
        radio.read(&buf, 1);
        return buf;
      }
      return -1;
    }

    virtual int peek() {
      return 0;
    }

    virtual void flush() {
      radio.flush_tx();
    }

    virtual size_t write(uint8_t iBuf) {
      size_t result = 1;
      radio.stopListening();
      if(!radio.write(&iBuf, 1)) {
        result = -1;
      }
      flush();
      radio.startListening();
      return result;
    }

  protected:
    RF24 &radio;
};
*/

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

//* WATCHDOG SETUP ********************************
void watchdogSetup() {
  watchdog.stop();
  delay(WATCHDOG_PERIOD);
  // watchdog.start(WATCHDOG_PERIOD);
  #if DEBUG
  Serial.println("Watchdog deployed");
  #endif
}

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
  watchdogSetup();
}

bool sendMsg()
{
  bool result = false;
  radio.stopListening();
  radio.flush_tx();
  uint8_t i = 0;
  if(!radio.write((const void *) &outgoing, (uint8_t) sizeof(Msg))) {
    radio.reUseTX();
  } else {
    result = true;
  }
  radio.startListening();
  return result;
}

void handleIncoming()
{
  switch(incoming.msgId) {
    case TELEM:
      digitalWrite(LED_PIN, 1);
      watchdog.feed();
      delay(5);
      digitalWrite(LED_PIN, 0);
      Telem &t = incoming.payload.telem;

      float front_ljx = ((float(t.ljx * 2) / 1023.0) - 1.0);
      float rear_ljx = front_ljx;
      
      float front_ljy = ((float(t.ljy * 2) / 1023.0) - 1.0);
      float rear_ljy = ljy;

      float rjx = ((float(t.rjx * 2) / 1023.0) - 1.0);
      front_ljx += rjx;
      rear_ljx -= rjx;
      
      float front_ljm = sqrt(front_ljx * front_ljx + front_ljy * front_ljy);
      float rear_ljm = sqrt(rear_ljx * rear_ljx + rear_ljy * rear_ljy);

      front_ljx /= front_ljm;
      front_ljy /= front_ljm;

      rear_ljx /= rear_ljm;
      rear_ljy /= rear_ljm;

      float frontHeading = atan2(front_ljy, front_ljx);
      float rearHeading = atan2(rear_ljy, rear_ljx);
      
      #if DEBUG
      Serial.println("(" + String(frontHeading) + ", " + String(rearHeading) + ")");
      #endif
      
      float frontLSpeed = sin(0.25 * PI + frontHeading) * front_ljm;
      float frontRSpeed = sin(-0.25 * PI + frontHeading) * front_ljm;
      float rearLSpeed = sin(-0.25 * PI + rearHeading) * rear_ljm;
      float rearRSpeed = sin(0.25 * PI + rearHeading) * rear_ljm;

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
      /*
      #if DEBUG
      Serial.println("Received telem: " + String(incoming.payload.telem.heading));
      #endif
      */
      break;
  }
}

uint32_t currentTime;

//* RF LOOP ****************************************************
void rfLoop() {
  if(radio.available()) {
    radio.read(((uint8_t *) &incoming), sizeof(Msg));
    handleIncoming();
  }

  if((currentTime - msgThrottleTime) > MSG_THROTTLE_PERIOD) {
    msgThrottleTime = currentTime;
  }

  if((currentTime - pingThrottleTime) > PING_PERIOD) {
    outgoing.msgId = PING;
    outgoing.payload.ping.pingCount = pingCount;
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
  }
}

void loop() {
  currentTime = millis();
  rfLoop();
  motorLoop();
}
