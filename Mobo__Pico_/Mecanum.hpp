#ifndef __MECANUM_HPP__
#define __MECANUM_HPP__

#include "VTask.hpp"
#include "VPWM.hpp"
#include "Wheel.hpp"

namespace robot
{
  #define RADPS_TO_RPM 9.5492965855

  class Mecanum : public VTask
  {
    public:
      // Constructor
      Mecanum(VPWM &iFLPWM, VPWM &iFRPWM, VPWM &iBLPWM, VPWM &iBRPWM, int iFLDirPin, int iFRDirPin, int iBLDirPin, int iBRDirPin, float iWidth, float iLength, float iWheelRadius)
      : VTask(),
        frontLeftPWM(iFLPWM),   // PWM for the front-left wheel
        frontRightPWM(iFRPWM),  // PWM for the front-right wheel
        backLeftPWM(iBLPWM),    // PWM for the back-left wheel
        backRightPWM(iBRPWM),   // PWM for the back-right wheel
        frontLeftSmoother(20),  // Smoother for the front-left encoder pulse count
        frontRightSmoother(20), // Smoother for the front-right encoder pulse count
        backLeftSmoother(20),   // Smoother for the back-left encoder pulse count
        backRightSmoother(20),  // Smoother for the back-right encoder pulse count

        // Front-left wheel construction
        frontLeftWheel(Wheel::FRONT_LEFT, frontLeftPWM, iFLDirPin, 0, frontLeftSmoother),
        // Front-right wheel construction
        frontRightWheel(Wheel::FRONT_RIGHT, frontRightPWM, iFRDirPin, 1, frontRightSmoother),
        // Back-left wheel construction
        backLeftWheel(Wheel::BACK_LEFT, backLeftPWM, iBLDirPin, 2, backLeftSmoother),
        // Back-right wheel construction
        backRightWheel(Wheel::BACK_RIGHT, backRightPWM, iBRDirPin, 3, backRightSmoother),

        // Precompute linear speed multiplier, and angular speed multiplier
        // incorporating a radians/second to revolutions/minute conversion
        linearSpeedFactor(RADPS_TO_RPM / iWheelRadius), angularSpeedFactor(0.5 * (iWidth + iLength))
      { }

      // Returns a Wheel reference given an ORIENTATION
      Wheel &operator[](Wheel::ORIENTATION iWhichWheel) {
        switch(iWhichWheel) {
          case Wheel::FRONT_RIGHT: return frontRightWheel;
          case Wheel::BACK_LEFT: return backLeftWheel;
          case Wheel::BACK_RIGHT: return backRightWheel;
          default: break;
        }
        return frontLeftWheel;
      }

      // Set wheel angular speeds given a heading vector (m/s)
      // and a rotational speed (radians/s)
      // https://www.researchgate.net/publication/269294739_Design_and_development_of_an_autonomous_omni-directional_mobile_robot_with_Mecanum_wheels
      void set(float iForward, float iLateral, float iAngular) {
        // Rather than attempting to clamp the input values, calculate the
        // ideal wheel speeds, and then clamp them to real world constraints

        // Calculate all the wheel speeds in revolutions per minute
        iForward *= linearSpeedFactor;
        iLateral *= linearSpeedFactor;
        iAngular *= linearSpeedFactor * angularSpeedFactor;
        float
          flRPM = (iForward - iLateral - iAngular),
          frRPM = (iForward + iLateral + iAngular),
          blRPM = (iForward + iLateral - iAngular),
          brRPM = (iForward - iLateral + iAngular);

        // Scale so the highest value is clamped to the maximum RPM
        float max = flRPM;
        if(frRPM > max) max = frRPM;
        if(blRPM > max) max = blRPM;
        if(brRPM > max) max = brRPM;
        if(max > Wheel::maxRPM) {
          float clamp = Wheel::maxRPM / max;
          flRPM *= clamp;
          frRPM *= clamp;
          blRPM *= clamp;
          brRPM *= clamp;
        }

        // Submit the RPM to each wheel
        frontLeftWheel.setRPM(flRPM);
        frontRightWheel.setRPM(frRPM);
        backLeftWheel.setRPM(blRPM);
        backRightWheel.setRPM(brRPM);
      }

      virtual void execute(float iElapsedTime) {
        frontLeftWheel.update(iElapsedTime);
        frontRightWheel.update(iElapsedTime);
        backLeftWheel.update(iElapsedTime);
        backRightWheel.update(iElapsedTime);
      }

    protected:
      VPWM
        &frontLeftPWM,
        &frontRightPWM,
        &backLeftPWM,
        &backRightPWM;

      TSmooth<float>
        frontLeftSmoother,
        frontRightSmoother,
        backLeftSmoother,
        backRightSmoother;

      Wheel
        frontLeftWheel,
        frontRightWheel,
        backLeftWheel,
        backRightWheel;

      float linearSpeedFactor;
      float angularSpeedFactor;
  };

}

#endif // __MECANUM_HPP__
