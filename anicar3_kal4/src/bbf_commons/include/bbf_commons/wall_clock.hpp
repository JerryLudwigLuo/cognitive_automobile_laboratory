// File:           wall_clock.hpp
// Creation Date:  Tuesday, April 27 2010
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(WALL_CLOCK_HPP)
#define WALL_CLOCK_HPP

#include <vector>
#include <iostream>
#include <sys/time.h>
#include <chrono>

#define SECS_PER_HOUR 3600
#define MSECS_PER_HOUR 3600000
#define NSECS_PER_HOUR 3600000000000
#define SECS_PER_MINUTE 60
#define MSECS_PER_MINUTE 60000
#define NSECS_PER_MINUTE 60000000000
#define MSECS_PER_SEC 1000
#define NSECS_PER_SEC 1000000000
#define NSECS_PER_MSEC 1000000

inline void toHoursMinutesSecondsMilliSeconds(
        const uint64_t& d,
        uint64_t& h,
        uint64_t& m,
        uint64_t& s,
        double& ms) {
    s = d / NSECS_PER_SEC;
    ms = static_cast<double>(d % NSECS_PER_SEC) / 1.e6;
    h = s / SECS_PER_HOUR;
    s %= SECS_PER_HOUR;
    m = s / SECS_PER_MINUTE;
    s %= SECS_PER_MINUTE;

}

inline uint64_t sumUpDurations(const std::vector<uint64_t>& durations) {
    uint64_t s = 0ul;
    for(const auto& d:durations) s += d;
    return s;
}

inline double wall_clock()
{
  timeval now;
  gettimeofday( &now, 0 );
  return now.tv_sec + now.tv_usec / 1000000.;
}

class wall_timer
{
public:
  wall_timer() { reset(); }
  void reset() { t = wall_clock(); }
  double elapsed() { return wall_clock() - t; }

private:
  double t;
};

class time_measure {
private:
    using clock = std::chrono::high_resolution_clock;

public:
    time_measure() : t1_(clock::now()) {
    }

    void restart() {
        t1_ = clock::now();
    }

    template<typename Unit = std::chrono::nanoseconds>
    uint64_t elapsed() const {
        return std::chrono::duration_cast<Unit>(clock::now() - t1_).count() ;
    }

    void printElapsedTime(const std::string &msg) const {
        const auto d = elapsed();
        uint64_t h, m, s;
        double ms;
        toHoursMinutesSecondsMilliSeconds(d, h, m, s, ms);
        std::cout << "Time elapsed: " << msg  << " => "
                  << h << "h " << m << "m " << s << "s " << ms  << "ms" << std::endl;
    }

    friend std::ostream& operator << (std::ostream& os, time_measure const &tm) {
        const auto d = tm.elapsed();
        uint64_t h, m, s;
        double ms;
        toHoursMinutesSecondsMilliSeconds(d, h, m, s, ms);
        return os << "Time elapsed: " << h << "h " << m << "m " << s << "s " << ms  << "ms";
    }

private:
    clock::time_point t1_;
};

#endif
