#include "PulseCounter.hpp"
#include <Arduino.h>

namespace robot
{
  VMutex *PulseCounter::sync = 0;
  unsigned long PulseCounter::reg[MAX_COUNTERS];

  #if MAX_COUNTERS > 0
  void irq1() { PulseCounter::reg[0]++; }
  #if MAX_COUNTERS > 1
  void irq2() { PulseCounter::reg[1]++; }
  #if MAX_COUNTERS > 2
  void irq3() { PulseCounter::reg[2]++; }
  #if MAX_COUNTERS > 3
  void irq4() { PulseCounter::reg[3]++; }
  #if MAX_COUNTERS > 4
  void irq5() { PulseCounter::reg[4]++; }
  #if MAX_COUNTERS > 5
  void irq6() { PulseCounter::reg[5]++; }
  #if MAX_COUNTERS > 6
  void irq7() { PulseCounter::reg[6]++; }
  #if MAX_COUNTERS > 7
  void irq8() { PulseCounter::reg[7]++; }
  #if MAX_COUNTERS > 8
  void irq9() { PulseCounter::reg[8]++; }
  #if MAX_COUNTERS > 9
  void irq10() { PulseCounter::reg[9]++; }
  #if MAX_COUNTERS > 10
  void irq11() { PulseCounter::reg[10]++; }
  #if MAX_COUNTERS > 11
  void irq12() { PulseCounter::reg[11]++; }
  #if MAX_COUNTERS > 12
  void irq13() { PulseCounter::reg[12]++; }
  #if MAX_COUNTERS > 13
  void irq14() { PulseCounter::reg[13]++; }
  #if MAX_COUNTERS > 14
  void irq15() { PulseCounter::reg[14]++; }
  #if MAX_COUNTERS > 16
  void irq16() { PulseCounter::reg[16]++; }
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif
  #endif

  void (*irq[MAX_COUNTERS])() = {
    #if MAX_COUNTERS > 0
    irq1
    #if MAX_COUNTERS > 1
    , irq2
    #if MAX_COUNTERS > 2
    , irq3
    #if MAX_COUNTERS > 3
    , irq4
    #if MAX_COUNTERS > 4
    , irq5
    #if MAX_COUNTERS > 5
    , irq6
    #if MAX_COUNTERS > 6
    , irq7
    #if MAX_COUNTERS > 7
    , irq8
    #if MAX_COUNTERS > 8
    , irq9
    #if MAX_COUNTERS > 9
    , irq10
    #if MAX_COUNTERS > 10
    , irq11
    #if MAX_COUNTERS > 11
    , irq12
    #if MAX_COUNTERS > 12
    , irq13
    #if MAX_COUNTERS > 13
    , irq14
    #if MAX_COUNTERS > 14
    , irq15
    #if MAX_COUNTERS > 15
    , irq16
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
  };

  void PulseCounter::init(unsigned char iPinCount, int *iPinArray, VMutex *iSync)
  {
    PulseCounter::sync = iSync;
    if(iPinCount > MAX_COUNTERS) iPinCount = MAX_COUNTERS;
    while(iPinCount--) {
      PulseCounter::reg[iPinCount] = 0;
      attachInterrupt(digitalPinToInterrupt(iPinArray[iPinCount]), irq[iPinCount], CHANGE);
    }
  }

  unsigned long PulseCounter::getAndResetCount(unsigned char iIndex)
  {
    unsigned long count;
    if(PulseCounter::sync) PulseCounter::sync->lock();
    count = PulseCounter::reg[iIndex];
    PulseCounter::reg[iIndex] = 0;
    if(PulseCounter::sync) PulseCounter::sync->unlock();
    return count;
  }
  
  PulseCounter PulseCounter::shared;
}
