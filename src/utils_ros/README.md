# utils_ros

Awesome helpers for using ROS.

## message_interpolation

## node_handle Helpers

## ros_console

## time_conversions

## smart_subscriber
A subscriber that only subscribes (or calls callbacks) if someone subscribes to one of the publishers passed as argument, avoiding any overhead when the results of a node are not needed.
The subscribing is done via an internal ros callback, so that the node starts working instantly when someone subscribes to the topic, so that is not noticable the node has not been working.

The use of the SmartSubscriber is only recommended when the node has no internal state (like e.g. a kalman filter), and only relays on simple input->output relations.

### Debugging
Because the node only connects to the input topics when someone subscribes, may not be shown by `rqt_graph`, `roswtf` or `rosnode`. To debug inactive connections, you can `export NO_SMART_SUBSCRIBE=1`.
In this case the smart subscriber will always subscribe so that all connections are visible.

### Example
A node subscribing to a Header topic and offering two header topics and one image topic. The callback (MyNode::msg_callback) will only be called if any of the three advertised topics has at least one subscriber.
```c++
/// my_node.h
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <utils_ros/smart_subscriber.hpp>
class MyNode {
public:
    MyNode(ros::NodeHandle, ros::NodeHandle);

private:
    // Message callback
    void msg_callback(const std_msgs::Header::ConstPtr& msg);

    // Publisher and subscriber
    ros::Publisher publisher1_;
    ros::Publisher publisher2_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_publisher_; // This is a special case: image transport has its own publisher
    utils_ros::SmartSubscriber<std_msgs::Header> subscriber_;
};


/// MyNode.cpp
MyNode::MyNode(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : image_transport_(private_node_handle){
    // Publisher
    publisher1_ = private_node_handle.advertise<std_msgs::Header>("topic_1", 5);
    publisher2_ = private_node_handle.advertise<std_msgs::Header>("topic_2", 5);

    // This callback is required for any non-standard publisher (like image_transport's)
    image_transport::SubscriberStatusCallback cb = boost::bind(&utils_ros::SmartSubscriber<std_msgs::Header>::subscribeCallback, &subscriber_);
    image_publisher_ = image_transport_.advertise("img", 5, cb, cb);

    // Pass all publishers on which the subscriber should look for subscriptions
    subscriber_.addPublishers({publisher1_, publisher2_});
    // Add our non-standard publisher
    subscriber_.addCustomPublisher(image_publisher_);
    // Configure the subscriber
    subscriber_.subscribe(private_node_handle, "out", 5, ros::TransportHints().tcpNoDelay(true));
    // Pass your message callback (this should be done as last step so that your callback is not called before you are done configuring)
    subscriber_.registerCallback(boost::bind(&MyNode::msg_callback, this, _1));
}
```
