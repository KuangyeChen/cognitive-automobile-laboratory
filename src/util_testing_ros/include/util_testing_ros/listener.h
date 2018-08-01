#pragma once
#include <chrono>
#include <string>
#include <thread>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

namespace util_testing {
/**
 * @brief Can be used to subscribe to messages in a comfortable way. Blocks until a message is received.
 * Make sure:
 * - you are using an AsyncSpinner - otherwise you will never receive messages.
 * - a time has been published at least once - otherwise nodelet managers will not load nodelets
 */
template <typename MessageT>
class Listener {
public:
    explicit Listener(std::string topic, ros::NodeHandle pubNh = ros::NodeHandle()) {
        subscriber = pubNh.subscribe(topic, 5, &Listener<MessageT>::onMessage, this);
    }

    /**
     * @brief waitForMessage blocks until a message is received or timeout reached
     * @param timeout maximum time to wait
     * @return message, if received - otherwise nullptr.
     */
    boost::shared_ptr<MessageT> waitForMessage(const ros::Duration& timeout = ros::Duration(2)) {
        msg.reset();
        int waitcount = 0;
        while (waitcount < 20 && !msg && ros::ok()) {
            waitcount++;
            std::this_thread::sleep_for(std::chrono::nanoseconds(timeout.toNSec() / 20));
        }
        return msg;
    }
    ros::Subscriber subscriber;      //!< actual subscriber to the message
    boost::shared_ptr<MessageT> msg; //!< contains most recent message

private:
    void onMessage(boost::shared_ptr<MessageT> msg) {
        ROS_DEBUG_STREAM("Received message on topic " << subscriber.getTopic()); // NOLINT
        this->msg = msg;
    }
};
} // namespace util_testing
