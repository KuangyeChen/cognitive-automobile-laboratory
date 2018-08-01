#pragma once
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>

#include <utility>

namespace util_testing {
/**
 * @brief This class provides a simple way to fake the clock in ros
 */
class TimeController {
public:
    /**
     * @brief Construcor for a time controller instance. Please instanciate just one of those...
     * @param t_init Start time for the time controller
     * @param spin set this to true if you are not using an async spinner and dont want to call spinOnce()
     */
    explicit TimeController(ros::Time tInit = ros::Time(1), bool spin = false)
            : spin{spin}, time_now{std::move(tInit)} {
        ros::NodeHandle nh;

        bool useSimTime = nh.param("/use_sim_time", false);
        if (!useSimTime) {
            throw std::runtime_error("use_sim_time is not set. Can not control time!");
        }
        time_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
        publish();
    }

    /**
     * @brief advance time a fixed amount of time and publish it
     * @param delta time to advance (must be >0)
     */
    void advance(const ros::Duration& delta = ros::Duration(1)) {
        time_now += delta;
        publish();
        if (spin) {
            ros::spinOnce();
        }
    }

    /**
     * @brief publish forces TimeController to publish the current time again
     */
    void publish() {
        rosgraph_msgs::Clock c;
        c.clock = time_now;
        time_publisher.publish(c);
    }
    bool spin; //!< spin after each time advance?
    // NOLINTNEXTLINE
    ros::Time time_now; //!< current time
    // NOLINTNEXTLINE
    ros::Publisher time_publisher; //!< publisher used for publishing to the clock topic
};
} // namespace util_testing
