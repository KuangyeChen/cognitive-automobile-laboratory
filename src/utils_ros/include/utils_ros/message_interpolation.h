//
// Created by bandera on 09.03.16.
//

#pragma once
#include <ros/time.h>

namespace utils_ros {

/*
 * \brief This function takes a vector of stamped! messages and interpolates between the msgs, according to the accessor
 * function specified by means of a lambda function.
 * Examle:
 *  double y_intpolated = utils_ros::interpolate<double, geometry_msgs::PoseStamped>(                       // Specify
 * return type and message type
 *                                    trajectory.poses,
 *                                    [](const geometry_msgs::PoseStamped p) { return p.pose.position.y; },
 *                                      timestamp);
 */
template <typename M, typename T>
inline M interpolate(const std::vector<T>& msgs, std::function<M(const T)> f, ros::Time timestamp) {
    auto N = msgs.size();

    if (timestamp <= msgs.front().header.stamp)
        return f(msgs.front());
    if (timestamp >= msgs.back().header.stamp)
        return f(msgs.back());

    for (int i = 0; i < N - 1; i++) {
        if (timestamp < msgs[i + 1].header.stamp) {
            double frac =
                (timestamp - msgs[i].header.stamp).toSec() / (msgs[i + 1].header.stamp - msgs[i].header.stamp).toSec();
            return f(msgs[i]) + frac * (f(msgs[i + 1]) - f(msgs[i]));
        }
    }
}
} // namespace utils_ros