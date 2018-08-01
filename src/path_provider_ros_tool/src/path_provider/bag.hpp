#pragma once

#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace path_provider_ros_tool {

template <typename T>
std::vector<typename T::ConstPtr> fromBag(const std::string& file_name, const std::string& topic) {

    rosbag::Bag bag(file_name);
    std::vector<std::string> topics{topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<typename T::ConstPtr> out;
    for (auto const& m : view) {
        const typename T::ConstPtr s = m.instantiate<T>();
        if (s != NULL)
            out.emplace_back(s);
    }
    return out;
    bag.close();
}
}
