#ifndef __VPWM_HPP__
#define __VPWM_HPP__

namespace robot
{
  class VPWM
  // Subclass to encapsulate native PWM functionality
  {
    public:
      VPWM() : dutyCycle(0.0) { }
      // Must have a vtable because it has a pure virtual member function
      virtual ~VPWM() { }

      // Set the current duty cycle, override setNative() to
      // call native code
      void setDutyCycle(float iPercent) {
        dutyCycle = iPercent;
        setNative(dutyCycle);
      }

      // Returns current duty cycle
      float getDutyCycle() {
        return dutyCycle;
      }

    protected:
      // Pure virtual, must be overridden in a subclass
      virtual void setNative(float iDutyCycle) = 0;

    protected:
      float dutyCycle;
  };
}

#endif // __PWM_HPP__
