#pragma once

#include <stdlib.h>
#include <ros/node_handle.h>

namespace utils_ros {

inline std::string getNodeName(const ros::NodeHandle& privateNodeHandle) {
    std::string name_space = privateNodeHandle.getNamespace();
    std::stringstream tempString(name_space);
    std::string name;
    while (std::getline(tempString, name, '/')) {
        ;
    }
    return name;
}

template <typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& val) {
    if (!nodeHandle.getParam(key, val)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        std::exit(EXIT_FAILURE);
    }
}

template <typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& val, const T& defaultValue) {
    nodeHandle.param(key, val, defaultValue);
}

template <typename T>
inline T getParam(const ros::NodeHandle& nodeHandle, const std::string& key) {
    if (!nodeHandle.hasParam(key)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        std::exit(EXIT_FAILURE);
    }
    T val;
    nodeHandle.getParam(key, val);
    return val;
}

template <typename T>
inline T getParam(const ros::NodeHandle& nodeHandle, const std::string& key, const T& defaultValue) {
    if (!nodeHandle.hasParam(key)) {
        return defaultValue;
    }
    T val;
    nodeHandle.getParam(key, val);
    return val;
}

template <>
inline unsigned int getParam(const ros::NodeHandle& nodeHandle, const std::string& key) {
    if (!nodeHandle.hasParam(key)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        std::exit(EXIT_FAILURE);
    }
    int val;
    nodeHandle.getParam(key, val);

    if (val < 0) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is negative, but should be unsigned!");
        std::exit(EXIT_FAILURE);
    }
    return static_cast<unsigned int>(val);
}

} // namespace utils_ros
