#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <rosinterface_handler/diagnostic_subscriber.hpp>

class DummyUpdater : public diagnostic_updater::Updater {
public:
    virtual ~DummyUpdater() {
    }

    void force_update() {
        status_vec.clear();
        for (auto& task : getTasks()) {
            diagnostic_updater::DiagnosticStatusWrapper status;

            status.name = task.getName();
            status.level = 2;
            status.message = "No message was set";
            status.hardware_id = "none";

            task.run(status);
            status_vec.push_back(status);
        }
    }
    std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;
};

using DiagPub = rosinterface_handler::DiagnosedPublisher<geometry_msgs::PointStamped>;
using DiagSub = rosinterface_handler::DiagnosedSubscriber<geometry_msgs::PointStamped>;
class TestDiagnosedPubSub : public testing::Test {
protected:
    void SetUp() override {
        updater.status_vec.clear();
        pub = nh.advertise<geometry_msgs::PointStamped>("test_topic", 5);
        sub.subscribe(nh, "test_topic", 5);
        pub.minFrequency(10).maxTimeDelay(1);
        sub.minFrequency(10).maxTimeDelay(1);
        messageCounter = 0;
    }
    ros::NodeHandle nh;
    DummyUpdater updater;
    DiagPub pub{updater};
    DiagSub sub{updater};
    int messageCounter{0};
};

TEST_F(TestDiagnosedPubSub, publishAndReceiveOK) {
    auto onTimer = [this](ros::TimerEvent e) {
        this->messageCounter++;
        auto msg = boost::make_shared<geometry_msgs::PointStamped>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.force_update();
    ASSERT_GE(4, updater.status_vec.size());
    for (auto& status : updater.status_vec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, publishAndReceiveFail) {
    auto onTimer = [this](ros::TimerEvent e) {
        this->messageCounter++;
        auto msg = boost::make_shared<geometry_msgs::PointStamped>();
        msg->header.stamp = e.current_real - ros::Duration(1.5);
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.15), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.force_update();
    ASSERT_LE(2, updater.status_vec.size());
    for (auto& status : updater.status_vec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, overrideTopic) {
    this->sub.subscribe(nh, "new_topic", 5);
    this->pub = nh.advertise<geometry_msgs::PointStamped>("new_topic", 5);
    auto onTimer = [this](ros::TimerEvent e) {
        this->messageCounter++;
        auto msg = boost::make_shared<geometry_msgs::PointStamped>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.force_update();
    ASSERT_LE(2, updater.status_vec.size());
    for (auto& status : updater.status_vec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, assign) {
    this->pub = DiagPub(updater);
    auto onTimer = [this](ros::TimerEvent e) {
        this->messageCounter++;
        auto msg = boost::make_shared<geometry_msgs::PointStamped>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.force_update();
    ASSERT_LE(1, updater.status_vec.size());
}
