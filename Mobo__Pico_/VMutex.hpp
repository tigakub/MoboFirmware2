#ifndef __VMUTEX_HPP__
#define __VMUTEX_HPP__

namespace robot
{
  class VMutex
  // Pure virtual class: interface for native critical section implementations
  {
    public:
      // Override to wrap native synchronization
      virtual void lock() = 0;
      virtual void unlock() = 0;
  };
}

#endif
