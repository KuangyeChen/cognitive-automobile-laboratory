#pragma once
#include <cstdlib>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/publication.h>
#include <ros/publisher.h>
#include <ros/topic_manager.h>

namespace utils_ros {

/**
 * @brief Subscriber that only actually subscribes to a topic if someone subscribes to a publisher
 * This is useful to avoid overhead for computing results that no one actually cares for.
 * Because this subscriber internally unsubscribes from a topic, upstream nodes are able to stop
 * publishing useless results as well.
 *
 * The smart subscriber can also be used for synchronized subscription via message_filters::TimeSynchronizer or similar.
 *
 * Set the environment variable NO_SMART_SUBSCRIBE to 1 to disable smart subscriptions.
 *
 * Usage example:
 * @code
 * void messageCallback(const std_msgs::Header::ConstPtr& msg) {
 * // do the work
 * }
 *
 * // subscribe in your main() or nodelet
 * ros::NodeHandle nh;
 * ros::Publisher myPub = nh.advertise<std_msgs::Header>("/output_topic", 5);
 * utils_ros::SmartSubscriber<std_msgs::Header> subscriber(nh, "/header_topic", 5, myPub);
 * subscriber.addCallback(messageCallback);
 * @endcode
 */
template <class Message>
class SmartSubscriber : public message_filters::Subscriber<Message> {
public:
    using Publishers = std::vector<ros::Publisher>;
    /**
     * @brief Default constuctor.
     * @param trackedPublishers publishers to check for subscriptions
     * Use the functions subscribe(), registerCallback() and addPublisher() to make this a full subscriber.
     */
    explicit SmartSubscriber(const Publishers& trackedPublishers = Publishers())
            : trackedPublishers_{trackedPublishers} {
        // check for always-on-mode
        const auto smart_subscribe = std::getenv("NO_SMART_SUBSCRIBE");
        try {
            if (smart_subscribe && std::stoi(smart_subscribe) > 0) {
                setSmart(false);
            }
        } catch (std::invalid_argument) {
        }

        // register suscribtion callbacks
        ros::SubscriberStatusCallback cb = boost::bind(&SmartSubscriber::subscribeCallback, this);

        callback_ =
            boost::make_shared<ros::SubscriberCallbacks>(cb, cb, ros::VoidConstPtr(), ros::getGlobalCallbackQueue());
        for (const auto& publisher : trackedPublishers_) {
            auto pub = ros::TopicManager::instance()->lookupPublication(publisher.getTopic());
            if (!pub)
                continue;
            pub->addCallbacks(callback_);
        }
    }

    /**
     * @brief Constructs a subscriber that monitors a single publisher for subscriptions
     * @param nh Node handle for subscribing
     * @param topic Topic to subscribe to
     * @param queue_size queue size of topic
     * @param trackedPublisher publisher to check for subscriptions
     * @param transport_hints Transport hints to pass along
     */
    SmartSubscriber(ros::NodeHandle& nh,
                    const std::string& topic,
                    uint32_t queue_size,
                    ros::Publisher& trackedPublisher,
                    const ros::TransportHints& transport_hints = ros::TransportHints())
            : SmartSubscriber(nh, topic, queue_size, Publishers{trackedPublisher}, transport_hints) {
    }

    /**
     * @brief Constructs a subscriber that monitors a single topic
     * @param nh Node handle for subscribing
     * @param topic Topic to subscribe to
     * @param queue_size queue size of topic
     * @param trackedPublisher publisher to check for subscriptions
     * @param transport_hints Transport hints to pass along
     */
    SmartSubscriber(ros::NodeHandle& nh,
                    const std::string& topic,
                    uint32_t queue_size,
                    const Publishers& trackedPublishers,
                    const ros::TransportHints& transport_hints = ros::TransportHints())
            : SmartSubscriber(trackedPublishers) {
        // call parent's subscribe
        this->subscribe(nh, topic, queue_size, transport_hints, nullptr);

        // monitor subscriber
        subscribeCallback();
    }

    ~SmartSubscriber() {
        for (const auto& publisher : trackedPublishers_) {
            auto pub = ros::TopicManager::instance()->lookupPublication(publisher.getTopic());
            // publisher might already be removed from topic manager
            if (!pub)
                continue;
            pub->removeCallbacks(callback_);
        }
    }

    /**
     * @brief Subscribe to a topic.
     *
     * Calls the message_filtes::Subscriber's subscribe internally.
     *
     * @param nh The ros::NodeHandle to use to subscribe.
     * @param topic The topic to subscribe to.
     * @param queue_size The subscription queue size
     * @param transport_hints The transport hints to pass along
     * @param callback_queue The callback queue to pass along
     */
    void subscribe(ros::NodeHandle& nh,
                   const std::string& topic,
                   uint32_t queue_size,
                   const ros::TransportHints& transport_hints = ros::TransportHints(),
                   ros::CallbackQueueInterface* callback_queue = 0) {
        message_filters::Subscriber<Message>::subscribe(nh, topic, queue_size, transport_hints, callback_queue);
        subscribeCallback();
    }

    using message_filters::Subscriber<Message>::subscribe;

    /**
     * @brief Convenience function to add new publishers to monitor
     * @param publishers publisher to look after
     */
    void addPublishers(const Publishers& publishers) {
        for (const auto& pub : publishers) {
            addPublisher(pub);
        }
    }

    /**
     * @brief Adds a new publisher to monitor
     * @param publisher to look after
     * Does nothing if pulisher is not valid
     */
    void addPublisher(ros::Publisher publisher) {
        auto pub = ros::TopicManager::instance()->lookupPublication(publisher.getTopic());
        // might not be subscribed
        if (!pub)
            return;
        pub->addCallbacks(callback_);
        trackedPublishers_.push_back(publisher);

        // check for subscribe
        subscribeCallback();
    }

    /**
     * @brief adds a non-default publisher (like image_transport's one)
     * @param publisher publisher to add
     * This has some limitations:
     * 1: Make sure this publisher has been created by passing the subscribeCallback() of this object to the advertise
     * function
     * both as connect and as disconnect callback
     * 2: Unlike normal publishers, this publisher cannot be removed from tracking
     * 3: The Publisher must be copy-constructible and have a function "uint32_t getNumSubscribers()"
     */
    template <class Publisher>
    void addCustomPublisher(const Publisher& publisher) {
        auto new_pub = std::make_shared<Publisher>(publisher);
        trackedCustomPublishers_.push_back(new_pub);
        getNumSubscriberFcns_.push_back(std::bind(&Publisher::getNumSubscribers, new_pub));

        // check for subscribe
        subscribeCallback();
    }

    /**
     * @brief stops tracking a publisher.
     * Does nothing if the publisher does not exist.
     * @return true if publisher existed and was removed
     */
    bool removePublisher(std::string topic) {
        // remove from vector
        auto found = std::find_if(trackedPublishers_.begin(),
                                  trackedPublishers_.end(),
                                  [topic](const ros::Publisher& pub) { return topic == pub.getTopic(); });
        if (found == trackedPublishers_.end())
            return false;
        trackedPublishers_.erase(found);
        auto pub = ros::TopicManager::instance()->lookupPublication(topic);
        if (!pub)
            return true;
        pub->removeCallbacks(callback_);
        return true;
    }

    /**
     * @brief returns whether this subsciber is currently subscribed to something
     * @return true if subscribed
     */
    bool isSubscribed() const {
        return (void*)this->getSubscriber();
    }

    /**
     * @brief returns whether this subscriber is currently in smart mode
     * @return true if in smart mode
     * If the subscriber is not in smart mode, it will behave like a normal ros publisher and will always be subscribed
     */
    bool smart() const {
        return smart_;
    }

    /**
     * @brief enable/disable smart mode
     * @param smart new mode for subscriber
     */
    void setSmart(bool smart) {
        smart_ = smart;
        subscribeCallback();
    }

    /**
     * @brief pass this callback to all non-standard publisher that you have
     * @return subscriber callback of this SmartSubscriber
     */
    const ros::SubscriberCallbacksPtr callback() const {
        return callback_;
    }

    /**
     * @brief checks for new subscribers and subscribes or unsubscribes if anything changed.
     * This function is not supposed to be called actively, it is only here so that you can pass it as callback to any
     * special publisher
     * (like image transport)
     */
    void subscribeCallback() {
        const auto subscribed = isSubscribed();
        bool subscribe = !smart();
        for (const auto& publisher : trackedPublishers_) {
            subscribe |= publisher.getNumSubscribers() > 0;
        }
        for (const auto& getNumSubscriber : getNumSubscriberFcns_) {
            subscribe |= getNumSubscriber() > 0;
        }
        if (subscribe && !subscribed) {
            ROS_DEBUG_STREAM("Got new subscribers. Subscribing to " << this->getSubscriber().getTopic());
            this->subscribe();
        }
        if (!subscribe && subscribed) {
            ROS_DEBUG_STREAM("No subscribers found. Unsubscribing from " << this->getSubscriber().getTopic());
            this->unsubscribe();
        }
    }

private:
    std::vector<ros::Publisher> trackedPublishers_;
    std::vector<std::shared_ptr<void>> trackedCustomPublishers_;
    std::vector<std::function<uint32_t()>> getNumSubscriberFcns_;
    ros::SubscriberCallbacksPtr callback_;
    bool smart_{true};
};
} // namespace utils_ros
