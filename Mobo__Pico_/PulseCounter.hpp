#ifndef __PULSECOUNTER_HPP__
#define __PULSECOUNTER_HPP__

#include "VMutex.hpp"

namespace robot
{
  #ifndef MAX_COUNTERS
  #define MAX_COUNTERS 4
  #endif

  class PulseCounter
  // Class to provide interrupt based pulse counting for multiple pins
  // To customize the number of counters, define MAX_COUNTERS before
  // #including this file
  {
    public:
      static void init(unsigned char iPinCount, int *iPinArray, VMutex *iSync = 0);
      static unsigned long getCount(unsigned char i) { return PulseCounter::reg[i]; }
      static void reset(unsigned char i) { PulseCounter::reg[i] = 0; }

      static unsigned long getAndResetCount(unsigned char i);

    protected:
      static VMutex *sync;

    public:
      static PulseCounter shared;
      static unsigned long reg[MAX_COUNTERS];
  };
}

#endif
