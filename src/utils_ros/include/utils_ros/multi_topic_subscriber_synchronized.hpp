/*
 * Copyright 2016. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  Andre-Marcel Hellmund
 *
 */

#pragma once

#include <functional>
#include <memory>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace utils_ros {

struct SubscriberSyncConfiguration {
    SubscriberSyncConfiguration() {
        ;
    }
    SubscriberSyncConfiguration(std::vector<std::string> ms, int qsize) : msg_names(ms), msg_queue_size(qsize) {
        ;
    }
    std::vector<std::string> msg_names;
    int msg_queue_size;
};

/**
*  @class SubscriberSyncBase
*  @par
*
*  class for synchronizing up to 5 topics.
* enters callbakc with a vector of messages
* instantiate with the factory
* f.e. :
* subscribers_ = utils_ros::getSyncBaseFactory<ObjectsIn>(
        nh_private, get_config(), boost::bind(&ObjectToTopviewConverter::callbackSubscriber, this, _1));
*/
template <typename MsgT>
class MultiTopicSubsriberSynchronizedBase {
public: // struct and typedefs
    using SubscriberType = message_filters::Subscriber<MsgT>;
    using CallbackFunc = std::function<void(const std::vector<typename MsgT::ConstPtr>&)>;

public:
    MultiTopicSubsriberSynchronizedBase(ros::NodeHandle& node_handle,
                                        const SubscriberSyncConfiguration& config,
                                        CallbackFunc func)
            : callback(func) {
        // make subscribers
        for (auto it = config.msg_names.cbegin(); it != config.msg_names.cend(); ++it) {
            subscribers_.push_back(std::make_unique<SubscriberType>(node_handle, *it, config.msg_queue_size));
        }
    }

protected:
    std::vector<std::unique_ptr<SubscriberType>> subscribers_;
    ros::NodeHandle node_handle_;
    CallbackFunc callback;
};

template <typename T, uint64_t N>
class MultiTopicSubscriberSynchronized : public MultiTopicSubsriberSynchronizedBase<T> {
    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        throw std::runtime_error("In MultiTopicScubscriberSynchronized: not supported for the specified number of "
                                 "topics. The maximum is 5 topics at the moment");
    }
};

template <typename T>
class MultiTopicSubscriberSynchronized<T, 1> : public MultiTopicSubsriberSynchronizedBase<T> {

    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    using TPtr = typename T::ConstPtr;

public:
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        if (this->subscribers_.size() != 1) {
            throw std::runtime_error("Size mismatch: expected 1, got " + std::to_string(this->subscribers_.size()));
        }

        this->subscribers_[0]->registerCallback(boost::bind(&MultiTopicSubscriberSynchronized::process, this, _1));
    }

    void process(const TPtr& m1) {
        this->callback({m1});
    }
};

template <typename T>
class MultiTopicSubscriberSynchronized<T, 2> : public MultiTopicSubsriberSynchronizedBase<T> {

    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    using TPtr = typename T::ConstPtr;

public:
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        if (this->subscribers_.size() != 2) {
            throw std::runtime_error("Size mismatch: expected 2, got " + std::to_string(this->subscribers_.size()));
        }
        // setup synchronizer
        sync_ =
            std::make_unique<Synchronizer>(ApproximateTime(100), *(this->subscribers_[0]), *(this->subscribers_[1]));
        sync_->registerCallback(boost::bind(&MultiTopicSubscriberSynchronized::process, this, _1, _2));
    }

    void process(const TPtr& m1, const TPtr& m2) {
        this->callback({m1, m2});
    }

private:
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<T, T>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;

    std::unique_ptr<Synchronizer> sync_;
};

template <typename T>
class MultiTopicSubscriberSynchronized<T, 3> : public MultiTopicSubsriberSynchronizedBase<T> {

    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    using TPtr = typename T::ConstPtr;

public:
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        if (this->subscribers_.size() != 3) {
            throw std::runtime_error("Size mismatch: expected 3, got " + std::to_string(this->subscribers_.size()));
        }
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(
            ApproximateTime(100), *(this->subscribers_[0]), *(this->subscribers_[1]), *(this->subscribers_[2]));
        sync_->registerCallback(boost::bind(&MultiTopicSubscriberSynchronized::process, this, _1, _2, _3));
    }

    void process(const TPtr& m1, const TPtr& m2, const TPtr& m3) {
        this->callback({m1, m2, m3});
    }

private:
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<T, T, T>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;

    std::unique_ptr<Synchronizer> sync_;
};

template <typename T>
class MultiTopicSubscriberSynchronized<T, 4> : public MultiTopicSubsriberSynchronizedBase<T> {

    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    using TPtr = typename T::ConstPtr;

public:
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        if (this->subscribers_.size() != 4) {
            throw std::runtime_error("Size mismatch: expected 4, got " + std::to_string(this->subscribers_.size()));
        }
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(ApproximateTime(100),
                                               *(this->subscribers_[0]),
                                               *(this->subscribers_[1]),
                                               *(this->subscribers_[2]),
                                               *(this->subscribers_[3]));
        sync_->registerCallback(boost::bind(&MultiTopicSubscriberSynchronized::process, this, _1, _2, _3, _4));
    }

    void process(const TPtr& m1, const TPtr& m2, const TPtr& m3, const TPtr& m4) {
        this->callback({m1, m2, m3, m4});
    }

private:
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<T, T, T, T>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;

    std::unique_ptr<Synchronizer> sync_;
};

template <typename T>
class MultiTopicSubscriberSynchronized<T, 5> : public MultiTopicSubsriberSynchronizedBase<T> {

    using CallbackFunc = typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc;
    using TPtr = typename T::ConstPtr;

public:
    MultiTopicSubscriberSynchronized(ros::NodeHandle& node_handle,
                                     const SubscriberSyncConfiguration& config,
                                     CallbackFunc func)
            : MultiTopicSubsriberSynchronizedBase<T>(node_handle, config, func) {
        if (this->subscribers_.size() != 5) {
            throw std::runtime_error("Size mismatch: expected 5, got " + std::to_string(this->subscribers_.size()));
        }
        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(ApproximateTime(100),
                                               *(this->subscribers_[0]),
                                               *(this->subscribers_[1]),
                                               *(this->subscribers_[2]),
                                               *(this->subscribers_[3]),
                                               *(this->subscribers_[4]));
        sync_->registerCallback(boost::bind(&MultiTopicSubscriberSynchronized::process, this, _1, _2, _3, _4, _5));
    }

    void process(const TPtr& m1, const TPtr& m2, const TPtr& m3, const TPtr& m4, const TPtr& m5) {
        this->callback({m1, m2, m3, m4, m5});
    }

private:
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<T, T, T, T, T>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;

    std::unique_ptr<Synchronizer> sync_;
};

template <typename T>
std::unique_ptr<MultiTopicSubsriberSynchronizedBase<T>> getSyncBaseFactory(
    ros::NodeHandle& node_handle,
    const SubscriberSyncConfiguration& config,
    typename MultiTopicSubsriberSynchronizedBase<T>::CallbackFunc func) {

    using TPtr = typename T::ConstPtr;

    auto numTopics = config.msg_names.size();

    ROS_DEBUG_STREAM("Multi_topic_subscriber_synched: Subsribing " << numTopics << " topics");

    switch (numTopics) {
    case 1:
        return std::make_unique<MultiTopicSubscriberSynchronized<T, 1>>(
            node_handle, config, [func](const std::vector<TPtr>& m) { func(m); });
    case 2:
        return std::make_unique<MultiTopicSubscriberSynchronized<T, 2>>(
            node_handle, config, [func](const std::vector<TPtr>& m) { func(m); });
    case 3:
        return std::make_unique<MultiTopicSubscriberSynchronized<T, 3>>(
            node_handle, config, [func](const std::vector<TPtr>& m) { func(m); });
    case 4:
        return std::make_unique<MultiTopicSubscriberSynchronized<T, 4>>(
            node_handle, config, [func](const std::vector<TPtr>& m) { func(m); });
    case 5:
        return std::make_unique<MultiTopicSubscriberSynchronized<T, 5>>(
            node_handle, config, [func](const std::vector<TPtr>& m) { func(m); });
    default:
        throw std::runtime_error("Invalid number of cameras configured. The maximum is 5 at the moment.");
    }
}
} // namespace utils_ros
