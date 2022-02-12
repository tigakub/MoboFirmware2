#define VERSION "4.0.001b"

#include <SPI.h>
#include "RF24.h"

#define DEBUG false

#define LED_PIN 17

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

/*
const uint32_t SIGNATURE 0x6f626f6dL

class ByteBanger
{
  protected:
    typedef enum {
      IDL = 0,
      SIG = 1,
      LEN = 2,
      PAY = 3
    } Mode;
    
  public:
    ByteBanger(Stream &iStream)
    : stream(iStream), sendBuffer(NULL), sendIndex(0), sendCount(0), signature(0), signatureMask(0), recvMode(0), recvBuffer(NULL), recvIndex(0), recvCount(0)
    { }
    
    uint8_t bang() {
      uint8_t result = 0;
      switch(sendMode) {
        case IDL:
          if(sendBuffer != NULL) {
            sendIndex = 0;
            sendPtr = (const uint8_t *) &SIGNATURE
            sendCount = sizeof(uint32_t);
            sendMode = SIG;
          }
          break;
        case SIG:
          if(sendIndex < sendCount) {
            radio.write(sendPtr[sendIndex]);
            sendIndex++;
          } else {
            sendIndex = 0;
            sendPtr = (uint8_t *) &payLength;
            sendCount = sizeof(uint8_t);
            sendMode = LEN;
          }
          break;
        case LEN:
          if(sendIndex < sendCount) {
            radio.write(sendPtr[sendIndex]);
            sendIndex++;
          } else {
            sendIndex = 0;
            sendPtr = (uint8_t *) sendBuffer;
            sendCount = payLength;
            sendMode = PAY;
          }
        case PAY:
          if(sendIndex < sendCount) {
            radio.write(sendPtr[sendIndex]);
            sendIndex++;
          } else {
            sendIndex = 0;
            sendPtr = NULL;
            sendBuffer = NULL;
            sendCount = 0;
            sendMode = IDL;
            result |= 1
          }
      }
      switch(recvMode) {
        case IDL:
        case SIG:
        case LEN:
        case PAY:
      }
      if(sendIndex < sendCount) {
        if(!sending) {
          sending = true;
        }
        stream.write(sendBuffer[sendIndex]);
        sendIndex++;
        if(sendIndex == sendCount) {
          result |= 1;
          sending = false;
        }
      }
      if(recvIndex < recvCount) {
        
        if(!receiving) {
          signature = 0;
          signatureMask = 0;
          bool signatureFound = false;
          uint32_t signature = 0;
          while(!signatureFound) {
            int thisByte = stream.read();
            if((((thisByte & 0xff) << 8) | (lastByte & 0xff)) != ) {
              lastByte = thisByte;
            }
          }
        }
        stream.read(recvBuffer[recvIndex]);
        recvIndex++;
        if(recvIndex == recvCount) {
          result |= 2;
          receiving = false;
        }
      }
      return result;
    }

  protected:
    bool scanForSignature(uint32_t &ioSignature, uint8_t &ioIndex, uint32_t iIncomingByte) {
      uint32_t mask = (-1) << (8 * (4 - ioIndex));
      iIncomingByte &= 0xff;
      uint32_t shiftedByte = iIncomingByte;
      shiftedByte <= (8 * (4 - ioIndex));
      ioSignature |= shiftedByte;
      if(ioSignature == SIGNATURE & mask) {
        if(ioIndex == 4) return true;
        ioIndex++;
      } else {
        if(iIncomingByte == (SIGNATURE >> 24)) {
          ioIndex = 1;
          ioSignature = iIncomingByte << 24;
        } else {
          ioIndex = 0;
          ioSignature = 0;
        }
      }
      return false;
    }

  protected:
    Stream &stream;
    Mode sendMode;
    uint8_t *sendBuffer;
    uint8_t *sendPtr;
    uint8_t sendIndex;
    uint8_t sendCount;
    Mode recvMode;
    uint8_t *recvBuffer;
    uint8_t *recvPtr;
    uint8_t recvIndex;
    uint8_t recvCount;
};
*/
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

  watchdog.stop();
  delay(WATCHDOG_PERIOD);
  watchdog.start(WATCHDOG_PERIOD);
  #if DEBUG
  Serial.println("Watchdog deployed");
  #endif
}

bool sendMsg()
{
  bool result = false;
  radio.stopListening();
  // radio.flush_tx(); // Automatically called by stopListening
  uint8_t i = 0;
  radio.writeFast((const void *) &outgoing, (uint8_t) sizeof(Msg));
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
      #endif
      break;
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  if(radio.available()) {
    radio.read(((uint8_t *) &incoming), sizeof(Msg));
    handleIncoming();
  }

  if((currentTime - msgThrottleTime) > MSG_THROTTLE_PERIOD) {
    outgoing.msgId = TELEM;
    outgoing.payload.telem.ljx = analogRead(A0);
    outgoing.payload.telem.ljy = analogRead(A1);
    outgoing.payload.telem.rjx = analogRead(A2);
    outgoing.payload.telem.rjy = analogRead(A3);
    outgoing.payload.telem.heading++;
    if(!sendMsg()) {
      #if DEBUG
      Serial.println("Failed to send telem msg");
      #endif
    }
    msgThrottleTime = currentTime;
  }
}
