#include "smart_subscriber.hpp"
#include <thread>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "ros_console.hpp"

typedef std_msgs::Int32 Msg;
class TestNode {
public:
    TestNode() {
        ros::NodeHandle nh;
        pub = nh.advertise<Msg>("/output", 5);
        sub = nh.subscribe("/input", 5, &TestNode::msgCallback, this);
        smartSub.subscribe(nh, "/input", 5);
        smartSub.addPublisher(pub);
        smartSub.registerCallback(boost::bind(&TestNode::smartMsgCallback, this, _1));
    }
    int msg_count{0};
    int smart_msg_count{0};
    ros::Publisher pub;
    ros::Subscriber sub;
    utils_ros::SmartSubscriber<std_msgs::Int32> smartSub;

private:
    void msgCallback(const Msg::ConstPtr& msg) {
        msg_count++;
        pub.publish(msg);
    }

    void smartMsgCallback(const Msg::ConstPtr& msg) {
        smart_msg_count++;
    }
};
class ImageNode {
    using SmartSub = utils_ros::SmartSubscriber<sensor_msgs::Image>;

public:
    ImageNode() : transport(ros::NodeHandle()) {
        image_transport::SubscriberStatusCallback cb = boost::bind(&SmartSub::subscribeCallback, &smartSub);
        pub = transport.advertise("/img_out", 5, cb, cb);
        ros::NodeHandle nh;
        smartSub.subscribe(nh, "/img_in", 5);
        smartSub.addCustomPublisher(pub);
        smartSub.registerCallback(boost::bind(&ImageNode::imgCallback, this, _1));
    }
    int msg_count{0};
    int smart_msg_count{0};
    image_transport::ImageTransport transport;
    image_transport::Publisher pub;
    SmartSub smartSub;

private:
    void imgCallback(const sensor_msgs::Image::ConstPtr& msg) {
        msg_count++;
        pub.publish(msg);
    }
};

void cb(const sensor_msgs::Image& msg) {
}

TEST(SmartSubscriber, subscribeTests) {
    // subscribe to a topic twice (smart and non smart)
    // make sure the smart subscriber publishes no messages after rostopic echo stops listening
    TestNode node;
    // check we are subscribed
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EXPECT_TRUE(node.smartSub.isSubscribed());

    // the node subscribed to us will stop listening after ~2 sec
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EXPECT_NEAR(node.msg_count, node.smart_msg_count, 5);
    EXPECT_GT(node.smart_msg_count, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // by now surely no one will listen
    const int old_smart_msg_count = node.msg_count;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_FALSE(node.smartSub.isSubscribed());
    EXPECT_NEAR(old_smart_msg_count, node.smart_msg_count, 2);
}

TEST(SmartSubscriber, basicTests) {
    ros::NodeHandle nh;
    ros::Publisher myPub = nh.advertise<Msg>("/output", 5);
    ros::Publisher myPub2 = nh.advertise<Msg>("/output2", 5);
    utils_ros::SmartSubscriber<Msg> sub(nh, "/input", 5, myPub);
    EXPECT_TRUE(sub.removePublisher(myPub.getTopic()));
    EXPECT_FALSE(sub.removePublisher(myPub2.getTopic()));
    sub.addPublisher(myPub2);
    EXPECT_TRUE(sub.removePublisher(myPub2.getTopic()));
}

TEST(SmartSubscriber, nondefaultPublisher) {
    ImageNode node;
    EXPECT_FALSE(node.smartSub.isSubscribed());
    ros::NodeHandle nh;
    // check we subscribe
    {
        auto sub = nh.subscribe("/img_out", 5, cb);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ros::spinOnce();
        EXPECT_TRUE(node.smartSub.isSubscribed());
    }
    // check we are no longer subscribed
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(node.smartSub.isSubscribed());

    // force subscription
    node.smartSub.setSmart(false);
    EXPECT_FALSE(node.smartSub.smart());
    EXPECT_TRUE(node.smartSub.isSubscribed());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "smart_subscriber_test");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // The async spinner lets you publish and receive messages during the tests,
    // no need to call spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
