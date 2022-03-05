#ifndef __PICOPWM_HPP__
#define __PICOPWM_HPP__

////////////////////////////////////////////////////////////////////////////////////
// Wrapper class for RP2040_PWM
////////////////////////////////////////////////////////////////////////////////////

#include <RP2040_PWM.h>
#include "VPWM.hpp"

using namespace robot;

class PicoPWM : public VPWM
{
  public:
    PicoPWM(int iPWMPin, int iPWMFreq = 2000)
    : native(nullptr), pwmPin(iPWMPin), pwmFreq(iPWMFreq) {
      native = new RP2040_PWM(pwmPin, pwmFreq, 0, false);
    }
    virtual ~PicoPWM() {
      // RP2040_PWM has no destructor
      native = nullptr;
    }

  protected:
    virtual void setNative(float iDutyCycle) {
      native->setPWM(pwmPin, pwmFreq, iDutyCycle);
    }

  protected:
    RP2040_PWM *native;
    int pwmPin;
    int pwmFreq;
};

#endif
