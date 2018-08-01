#pragma once
#include <ros/ros.h>

namespace stargazer_ros_tool {

/**
 * Show summary about node containing name, namespace,
 * subscribed and advertised topics.
 */
inline void showNodeInfo() {

    using namespace ros::this_node;

    std::vector<std::string> subscribed_topics, advertised_topics;
    getSubscribedTopics(subscribed_topics);
    getAdvertisedTopics(advertised_topics);

    std::ostringstream msg_subscr, msg_advert;
    for (auto const& t : subscribed_topics) {
        msg_subscr << t << std::endl;
    }
    for (auto const& t : advertised_topics) {
        msg_advert << t << std::endl;
    }

    ROS_INFO_STREAM("Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl
                                << "Subscribed topics: " << std::endl
                                << msg_subscr.str() << "Advertised topics: " << std::endl
                                << msg_advert.str());
}

template <typename T>
inline void getParam(const ros::NodeHandle& node_handle, const std::string key, T& val) {

    if (!node_handle.getParam(key, val)) {
        ROS_ERROR_STREAM("Undefined parameter '" << key << "'.");
        std::exit(EXIT_FAILURE);
    }
}

} // namespace stargazer_ros_tool
