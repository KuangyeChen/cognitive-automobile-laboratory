#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <ros/ros.h>

namespace utils_ros {

template <typename MsgT>
class MultiTopicSubscriber {

    using Lock = std::lock_guard<std::mutex>;

public:
    using Ptr = std::unique_ptr<MultiTopicSubscriber>;
    using MsgVector = std::vector<typename MsgT::ConstPtr>;

public:
    inline MultiTopicSubscriber(const size_t& msg_queue_size = 5) : msg_queue_size_{msg_queue_size} {};

    inline MultiTopicSubscriber(ros::NodeHandle& nh,
                                const std::vector<std::string>& msg_names,
                                const size_t& msg_queue_size = 5)
            : msg_queue_size_{msg_queue_size} {
        for (auto const& msg_name : msg_names)
            addTopic(nh, msg_name);
    }

    inline MultiTopicSubscriber(ros::NodeHandle& nh,
                                const std::string& base_path,
                                const std::vector<std::string>& msg_names,
                                const size_t& msg_queue_size = 5)
            : msg_queue_size_{msg_queue_size} {
        for (auto const& msg_name : msg_names)
            addTopic(nh, base_path + "/" + msg_name);
    }

    inline void addTopic(ros::NodeHandle& nh, const std::string& msg_name) {
        subscribers_.emplace_back(nh.subscribe(
            msg_name, msg_queue_size_, &MultiTopicSubscriber::process, this, ros::TransportHints().tcpNoDelay()));
    }

    inline static MultiTopicSubscriber::Ptr create(const size_t& msg_queue_size = 5) {
        return std::make_unique<MultiTopicSubscriber>(msg_queue_size);
    }
    inline static MultiTopicSubscriber::Ptr create(ros::NodeHandle& nh,
                                                   const std::vector<std::string>& msg_names,
                                                   const size_t& msg_queue_size = 5) {
        return std::make_unique<MultiTopicSubscriber>(nh, msg_names, msg_queue_size);
    }
    inline static MultiTopicSubscriber::Ptr create(ros::NodeHandle& nh,
                                                   const std::string& base_path,
                                                   const std::vector<std::string>& msg_names,
                                                   const size_t& msg_queue_size = 5) {
        return std::make_unique<MultiTopicSubscriber>(nh, base_path, msg_names, msg_queue_size);
    }

    inline MsgVector& getMsgs() {
        return msg_list_;
    }
    inline const MsgVector& getMsgs() const {
        return msg_list_;
    }

    inline void clear() {
        Lock l{buffer_mutex_};
        msg_list_.clear();
    }

    inline void getMsgsAndClearBuffer(MsgVector& msgs) {
        Lock l{buffer_mutex_};
        msgs = msg_list_;
        msg_list_.clear();
    }

    inline MsgVector getMsgsAndClearBuffer() {
        MsgVector copy;
        getMsgsAndClearBuffer(copy);
        return copy;
    }

private:
    inline void process(const typename MsgT::ConstPtr& msg) {
        Lock l{buffer_mutex_};
        msg_list_.emplace_back(msg);
    }

    std::mutex buffer_mutex_;
    MsgVector msg_list_;
    std::vector<ros::Subscriber> subscribers_;
    size_t msg_queue_size_;
};
} // namespace utils_ros
