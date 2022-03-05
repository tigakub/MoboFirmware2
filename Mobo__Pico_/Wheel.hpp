#ifndef __WHEEL_HPP__
#define __WHEEL_HPP__

#include <Arduino.h>
#include <math.h>

#include "VPWM.hpp"
#include "TSmooth.hpp"
#include "VMutex.hpp"
#include "PulseCounter.hpp"

#define ONE_OVER_TWO_PI 0.1591549431

namespace robot
{
  class Wheel
  // Encapsulates Wheel behavior
  {
    public:
      // Signifies wheel position and orientation
      typedef enum {
        FRONT_LEFT = 1,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
      } ORIENTATION;

      // Signifies direction of rotation
      typedef enum {
        FORWARD = 1,
        BACKWARD = -1
      } DIRECTION;

    public:
      // Constructor
      Wheel(ORIENTATION iOrientation, VPWM &iPWM, int iDirPin, int iPulseCounterIndex, TSmooth<float> &iPulseCountSmoother);

      static void initStatics(int iBrkPin, float iMaxDutyCycle, float iMaxRPM, float iPulseToRPM) {
        pinMode(Wheel::brkPin, OUTPUT);
        Wheel::breakState = 0;
        Wheel::brkPin = iBrkPin;
        Wheel::maxDutyCycle = iMaxDutyCycle;
        Wheel::maxRPM = iMaxRPM;
        Wheel::pulseToRPM = iPulseToRPM;
        Wheel::rpmToDuty = Wheel::maxDutyCycle / Wheel::maxRPM;
        Wheel::initialized = true;
      }

      // Slams on the breaks
      static void setBreak() {
        digitalWrite(Wheel::brkPin, LOW);
        breakState = 0;
      }
      // Releases the breaks
      static void releaseBreak() {
        digitalWrite(Wheel::brkPin, HIGH);
        breakState = 1;
      }

      // Direct access to the associated VPWM object
      VPWM &getPWMObject() { return pwm; }

      // Set the RPM and direction; sets internal variables,
      // must call update() to put into effect
      void setRPM(float iRPM) {
        desiredRPM = fabs(iRPM);
        if(iRPM > 0) direction = FORWARD;
        else direction = BACKWARD;
      }

      // Convenience function to set RPM by specifying angular speed in
      // radians per second
      void setAngularSpeed(float iW) {
        float rps = iW * ONE_OVER_TWO_PI;
        setRPM(rps * 60.0);
      }

      // RPM accessor
      float getRPM() { return currentRPM; }

      // Should be called periodically to allow object to update
      // the PWM duty cycle and calculate the current RPM and
      // angular speed from the encoder pulse count
      void update(float iElapsedTime) {
        updatePWM();
        updateRPM(iElapsedTime);
      }

    protected:
      // Internal function to compute PWM duty cycle from desired RPM
      float deriveDutyCycle() {
        return desiredRPM * Wheel::rpmToDuty;
      }

      // Internal function that submits the duty cycle to the hardware
      void updatePWM();

      // Internal function that computes the current RPM from the
      // encoder pulse count
      void updateRPM(float iElapsedTime) {
        unsigned long tPulseCount = PulseCounter::shared.getAndResetCount(pulseCounterIndex);
        pulseCountSmoother.addValue(static_cast<float>(tPulseCount));
        float currentCount = pulseCountSmoother.getValue();
        currentRPM = Wheel::pulseToRPM * currentCount / iElapsedTime;
      }

    // Public statics need to be set before instantiating any wheels
    public:
      static int brkPin;
      static float pulseToRPM;
      static float maxRPM;
      static float maxDutyCycle;
      static float rpmToDuty;

    protected:
      static bool initialized;
      static int breakState;

      ORIENTATION orientation;
      VPWM &pwm;
      int pwmPin;
      int dirPin;

      float dutyCycle;
      int polarity;
      int direction;
      float currentRPM;
      float desiredRPM;

      int pulseCounterIndex;
      TSmooth<float> &pulseCountSmoother;
  };
}

#endif // __WHEEL_HPP__
