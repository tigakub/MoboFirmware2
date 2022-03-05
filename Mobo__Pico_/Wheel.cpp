#include "Wheel.hpp"

namespace robot
{
  bool Wheel::initialized = false;
  int Wheel::breakState;

  int Wheel::brkPin = 0;
  float Wheel::maxDutyCycle = 0.0;
  float Wheel::maxRPM = 0.0;
  float Wheel::rpmToDuty = 0.0;
  float Wheel::pulseToRPM = 0.0;

  Wheel::Wheel(ORIENTATION iOrientation, VPWM &iPWM, int iDirPin, int iPulseCounterIndex, TSmooth<float> &iPulseCountSmoother)
  : orientation(iOrientation), pwm(iPWM), dirPin(iDirPin),
    direction(FORWARD), currentRPM(0.0), desiredRPM(0.0), pulseCounterIndex(iPulseCounterIndex), pulseCountSmoother(iPulseCountSmoother)
  {
    // Set polarity of wheel based on orientation
    switch(orientation) {
      case FRONT_LEFT: polarity = 1; break;
      case BACK_LEFT: polarity = 1; break;
      case FRONT_RIGHT: polarity = -1; break;
      case BACK_RIGHT: polarity = -1; break;
    }

    // Initialize direction pin
    pinMode(dirPin, OUTPUT);
  }

  void Wheel::updatePWM()
  {
    if(Wheel::breakState) {
      pwm.setDutyCycle(deriveDutyCycle());
      digitalWrite(dirPin, (direction * polarity) < 0);
    } else {
      pwm.setDutyCycle(0);
    }
  }
}
