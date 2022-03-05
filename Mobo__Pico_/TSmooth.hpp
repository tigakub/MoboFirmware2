#ifndef __TSMOOTH_HPP__
#define __TSMOOTH_HPP__

namespace robot
{
  template <class T>
  class TSmooth
  // Template class to provide simple moving average smoothing
  {

    public:
      // Constructor
      TSmooth(int iWindow)
      : window(iWindow), tail(0), count(0), buffer(0), sum(0), smoothedValue(0) {
        // Allocate buffer for the cyclic fifo queue
        buffer = new T[window];
      }

      /*
      // Just assume that these will never be deleted, save memory by avoiding
      // a vtable
      // Destructor
      virtual ~TSmooth() {
        // Housekeeping
        delete [] buffer;
      }
      */

      // Updates moving average given a new value
      void addValue(T iNewValue) {
        // Default behavior is Simple Moving Average
        // If the buffer is not yet full
        (count < window)
          // Increment the count, and store its reciprocal
          ? (recip = 1.0 / static_cast<T>(++count))
          // Otherwise, pop the oldest value
          : sum -= buffer[tail];
        buffer[tail] = iNewValue;

        // Add the new value to the buffer
        sum += iNewValue;

        // Index the next position in the buffer
        tail++;

        // Wrap the tail around when it reaches the end
        tail %= window;

        // Calculate the new average
        smoothedValue = sum * recip;
      }

      // Returns the current moving average
      T getValue() {
        return smoothedValue;
      }

    protected:
      int window, tail, count;
      T recip;
      T *buffer;
      T sum;
      T smoothedValue;
  };
}

#endif // __VSMOOTH_HPP__
