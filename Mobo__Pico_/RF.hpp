#ifndef __RF_HPP__
#define __RF_HPP__

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "VTask.hpp"

namespace robot
{
  class RF : public VTask
  {
    public:
      class VMsg
      {
        public:
          VMsg() : msgId() { }
          virtual ~VMsg() { }

          virtual uint8_t size() = 0;

          uint32_t msgId;
      };

      class VHandler
      {
        public:
          virtual void operator()(VMsg &iMsg) = 0;
      };

    public:
      RF(uint64_t iWriteAddr, uint64_t iReadAddr, int iCEPin, int iCSPin, VMsg &iIncoming, VHandler &iHandler)
      : VTask(), radio(iCEPin, iCSPin), writeAddr(iWriteAddr), readAddr(iReadAddr),
        incoming(iIncoming), handler(iHandler) { }

      virtual void start(unsigned long iPeriod) {
        while(!radio.begin()) {
          delay(1000);
        }

        radio.setAutoAck(true);
        radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
        radio.enableAckPayload();
        radio.setRetries(0, 15);

        // save on transmission time by setting the radio to only transmit the
        // number of bytes we need to transmit
        radio.setPayloadSize(incoming.size());

        // set the TX address of the RX node into the TX pipe
        radio.openWritingPipe(writeAddr);

        // set the RX address of the TX node into a RX pipe
        radio.openReadingPipe(1, readAddr);

        VTask::start(iPeriod);
      }

      virtual bool sendMsg(VMsg &iMsg) {
        bool result = false;
        radio.stopListening();
        radio.flush_tx();
        uint8_t i = 0;
        if(!radio.write(static_cast<const void*>(&iMsg), static_cast<uint8_t>(iMsg.size()))) {
          radio.reUseTX();
        } else {
          result = true;
        }
        radio.startListening();
        return result;
      }

      virtual void execute(float iElapsedTime) {
        if(radio.available()) {
          radio.read(static_cast<void *>(&incoming), incoming.size());
          handler(incoming);
        }
      }

    protected:
      RF24 radio;
      uint64_t writeAddr, readAddr;

    public:
      VMsg &incoming;
      VHandler &handler;
  };
}

#endif // __RF_HPP__
