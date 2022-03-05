#ifndef __VTASK_HPP__
#define __VTASK_HPP__

namespace robot
{
  class VTask
  {
    public:
      VTask()
      : enable(false), period(0), lastTime(0) { }

      void start(unsigned long iPeriod) {
        period = iPeriod;
        lastTime = millis();
        enable = true;
      }

      void reset() {
        lastTime = millis();
      }

      void stop() {
        enable = false;
      }

      void yield(unsigned long iCurrentTime) {
        unsigned long elapsedMillis = iCurrentTime - lastTime;
        if(elapsedMillis > period) {
          float elapsedTime = static_cast<float>(elapsedMillis) * 0.001;
          execute(elapsedTime);
          lastTime = iCurrentTime;
        }
      }

      virtual void execute(float iElapsedTime) = 0;

    protected:
      bool enable;
      unsigned long period;
      unsigned long lastTime;
  };
}

#endif // __VTASK_HPP__
