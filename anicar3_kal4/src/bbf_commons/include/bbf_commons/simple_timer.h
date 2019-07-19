#ifndef SIMPLE_TIMER_H
#define SIMPLE_TIMER_H

#include <chrono>

struct timer
{
  typedef std::chrono::system_clock Clock;
  typedef std::chrono::milliseconds milliseconds;

  Clock::time_point t0, t1;

  void start()
  {
    t0 = Clock::now();
  }

  void ellapsed(std::string context)
  {
    milliseconds ms = ellapsed();
    std::cout << context << " took " << ms.count() << "ms" << std::endl;
  }

  milliseconds ellapsed()
  {
      t1 = Clock::now();
      milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
      return ms;
  }

  void tic()
  {
      start();
  }

  milliseconds toc()
  {
      return ellapsed();
  }

  milliseconds toctic()
  {
      milliseconds buffer = ellapsed();
      tic();
      return buffer;
  }

  milliseconds toctic( std::string context )
  {
      milliseconds _s = toctic();
      std::cout << "TIMER " << context  << " -> " << _s.count() << std::endl;
      return _s;
  }

};

#endif // SIMPLE_TIMER_H
