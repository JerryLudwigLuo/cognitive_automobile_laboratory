#include <chrono>

#include <ros/time.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace utils_ros {

template <typename T>
inline T toNanoSeconds(const ros::Time& time) {
    return static_cast<T>(time.toNSec());
}

template <typename T>
inline T toMicroSeconds(const ros::Time& time) {
    return static_cast<T>(time.toNSec() * 1e-3);
}

template <typename T>
inline T toSeconds(const ros::Time& time) {
    return static_cast<T>(time.toSec());
}

template <typename T>
inline T toNanoSeconds(const ros::Duration& duration) {
    return static_cast<T>(duration.toNSec());
}

template <typename T>
inline T toMicroSeconds(const ros::Duration& duration) {
    return static_cast<T>(duration.toNSec() * 1e-3);
}

template <typename T>
inline T toSeconds(const ros::Duration& duration) {
    return static_cast<T>(duration.toSec());
}

inline ros::Time toRosTime(const std::chrono::system_clock::time_point& from) {
    auto d = std::chrono::duration_cast<std::chrono::nanoseconds>(from.time_since_epoch()).count();
    auto sec = d / 1000000000;
    auto nsec = d % 1000000000;
    const boost::posix_time::ptime p = boost::posix_time::from_time_t(0) +
                                       boost::posix_time::seconds(static_cast<long>(sec)) +
#ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
                                       boost::posix_time::nanoseconds(nsec);
#else
                                       boost::posix_time::microseconds((nsec + 500) / 1000);
#endif
    return ros::Time::fromBoost(p);
}

} // namespace utils_ros
