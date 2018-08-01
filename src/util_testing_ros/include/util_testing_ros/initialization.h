#pragma once
#include <chrono>
#include <initializer_list>
#include <thread>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/publisher.h>

namespace util_testing {
/**
 * @brief waitForInitalization blocks until a node has subscribed to all publishers
 * @param publishers list with all publishers {publisher1, publisher2, ..}
 * @param timeout time to wait for nodes to subscribe
 * @return true if nodes managed to subscribe, false after timeout
 */
inline bool waitForInitalization(const std::initializer_list<ros::Publisher>& publishers,
                                 const ros::Duration& timeout = ros::Duration(5)) {
    int waitcount = 0;
    while (waitcount++ < 20 && ros::ok()) {
        bool online = true;
        for (const ros::Publisher& publisher : publishers) {
            online &= publisher.getNumSubscribers() >= 1;
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(timeout.toNSec() / 20));
        if (online) {
            return true;
        }
    }
    return false;
}
} // namespace util_testing
