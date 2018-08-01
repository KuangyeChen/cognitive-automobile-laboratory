#include <ros/time.h>

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
} // namespace utils_ros
