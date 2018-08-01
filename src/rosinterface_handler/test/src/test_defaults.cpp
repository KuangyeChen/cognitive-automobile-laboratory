#include <gtest/gtest.h>
#include <rosinterface_handler/DefaultsInterface.h>

typedef rosinterface_handler::DefaultsInterface IfType;
typedef rosinterface_handler::DefaultsConfig ConfigType;

TEST(RosinterfaceHandler, DefaultParams) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer());

    ASSERT_EQ("info", testInterface.verbosity_param_w_default);

    ASSERT_EQ(1, testInterface.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testInterface.double_param_w_default);
    ASSERT_EQ("Hello World", testInterface.str_param_w_default);
    ASSERT_EQ(true, testInterface.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testInterface.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testInterface.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testInterface.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testInterface.vector_string_param_w_default);

    std::map<std::string, std::string> tmp{{"Hello", "World"}};
    ASSERT_EQ(tmp, testInterface.map_param_w_default);

    ASSERT_EQ(1, testInterface.enum_int_param_w_default);
    ASSERT_EQ("One", testInterface.enum_str_param_w_default);
}

TEST(RosinterfaceHandler, DefaultSubscriber) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer());

    ASSERT_TRUE(!!testInterface.subscriber_w_default);
    ASSERT_EQ(testInterface.subscriber_w_default->getTopic(), "/test/rosinterface_handler_test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_public_w_default);
    ASSERT_EQ(testInterface.subscriber_public_w_default->getTopic(), "/test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_global_w_default);
    ASSERT_EQ(testInterface.subscriber_global_w_default->getTopic(), "/in_topic");
}

TEST(RosinterfaceHandler, DefaultPublisher) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer());

    ASSERT_EQ(testInterface.publisher_w_default.getTopic(), "/test/rosinterface_handler_test/out_topic");
    ASSERT_EQ(testInterface.publisher_public_w_default.getTopic(), "/test/out_topic");
    ASSERT_EQ(testInterface.publisher_global_w_default.getTopic(), "/out_topic");
}

TEST(RosinterfaceHandler, DefaultsOnParamServer) {
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer());

    // values should now be set on parameter server
    {
        std::string verbosity;
        ASSERT_TRUE(nh.getParam("verbosity_param_w_default", verbosity));
        EXPECT_EQ(verbosity, testInterface.verbosity_param_w_default);
    }
    {
        int int_interface;
        ASSERT_TRUE(nh.getParam("int_param_w_default", int_interface));
        ASSERT_EQ(int_interface, testInterface.int_param_w_default);
    }
    {
        double double_interface;
        ASSERT_TRUE(nh.getParam("double_param_w_default", double_interface));
        EXPECT_EQ(double_interface, testInterface.double_param_w_default);
    }
    {
        bool bool_interface;
        ASSERT_TRUE(nh.getParam("bool_param_w_default", bool_interface));
        EXPECT_EQ(bool_interface, testInterface.bool_param_w_default);
    }
    {
        std::string string_interface;
        ASSERT_TRUE(nh.getParam("str_param_w_default", string_interface));
        EXPECT_EQ(string_interface, testInterface.str_param_w_default);
    }
    {
        std::vector<int> vector_int_interface;
        ASSERT_TRUE(nh.getParam("vector_int_param_w_default", vector_int_interface));
        EXPECT_EQ(vector_int_interface, testInterface.vector_int_param_w_default);
    }
    {
        std::vector<double> vector_double_interface;
        ASSERT_TRUE(nh.getParam("vector_double_param_w_default", vector_double_interface));
        EXPECT_EQ(vector_double_interface, testInterface.vector_double_param_w_default);
    }
    {
        std::vector<bool> vector_bool_interface;
        ASSERT_TRUE(nh.getParam("vector_bool_param_w_default", vector_bool_interface));
        EXPECT_EQ(vector_bool_interface, testInterface.vector_bool_param_w_default);
    }
    {
        std::vector<std::string> vector_string_interface;
        ASSERT_TRUE(nh.getParam("vector_string_param_w_default", vector_string_interface));
        EXPECT_EQ(vector_string_interface, testInterface.vector_string_param_w_default);
    }
    {
        std::map<std::string, std::string> map_param_w_default;
        ASSERT_TRUE(nh.getParam("map_param_w_default", map_param_w_default));
        EXPECT_EQ(map_param_w_default, testInterface.map_param_w_default);
    }
    {
        int enum_int_interface;
        ASSERT_TRUE(nh.getParam("enum_int_param_w_default", enum_int_interface));
        EXPECT_EQ(enum_int_interface, testInterface.enum_int_param_w_default);
    }
    {
        std::string enum_str_interface;
        ASSERT_TRUE(nh.getParam("enum_str_param_w_default", enum_str_interface));
        EXPECT_EQ(enum_str_interface, testInterface.enum_str_param_w_default);
    }
}

TEST(RosinterfaceHandler, SetParamOnServer) {
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer());

    testInterface.verbosity_param_w_default = "warning";
    testInterface.int_param_w_default = 2;
    testInterface.double_param_w_default = 2.2;
    testInterface.str_param_w_default = "World Hello";
    testInterface.bool_param_w_default = false;
    testInterface.vector_int_param_w_default = std::vector<int>{3, 2, 1};
    testInterface.vector_double_param_w_default = std::vector<double>{1.3, 1.2, 1.2};
    testInterface.vector_bool_param_w_default = std::vector<bool>{true, false};
    testInterface.vector_string_param_w_default = std::vector<std::string>{"World", "Hello"};
    testInterface.map_param_w_default = std::map<std::string, std::string>{{"World", "Hello"}};
    testInterface.enum_int_param_w_default = 2;
    testInterface.enum_str_param_w_default = "Two";

    testInterface.toParamServer();

    // values should now be set on parameter server
    {
        std::string verbosity;
        ASSERT_TRUE(nh.getParam("verbosity_param_w_default", verbosity));
        EXPECT_EQ(verbosity, testInterface.verbosity_param_w_default);
    }
    {
        int int_interface;
        ASSERT_TRUE(nh.getParam("int_param_w_default", int_interface));
        ASSERT_EQ(int_interface, testInterface.int_param_w_default);
    }
    {
        double double_interface;
        ASSERT_TRUE(nh.getParam("double_param_w_default", double_interface));
        EXPECT_EQ(double_interface, testInterface.double_param_w_default);
    }
    {
        bool bool_interface;
        ASSERT_TRUE(nh.getParam("bool_param_w_default", bool_interface));
        EXPECT_EQ(bool_interface, testInterface.bool_param_w_default);
    }
    {
        std::string string_interface;
        ASSERT_TRUE(nh.getParam("str_param_w_default", string_interface));
        EXPECT_EQ(string_interface, testInterface.str_param_w_default);
    }
    {
        std::vector<int> vector_int_interface;
        ASSERT_TRUE(nh.getParam("vector_int_param_w_default", vector_int_interface));
        EXPECT_EQ(vector_int_interface, testInterface.vector_int_param_w_default);
    }
    {
        std::vector<double> vector_double_interface;
        ASSERT_TRUE(nh.getParam("vector_double_param_w_default", vector_double_interface));
        EXPECT_EQ(vector_double_interface, testInterface.vector_double_param_w_default);
    }
    {
        std::vector<bool> vector_bool_interface;
        ASSERT_TRUE(nh.getParam("vector_bool_param_w_default", vector_bool_interface));
        EXPECT_EQ(vector_bool_interface, testInterface.vector_bool_param_w_default);
    }
    {
        std::vector<std::string> vector_string_interface;
        ASSERT_TRUE(nh.getParam("vector_string_param_w_default", vector_string_interface));
        EXPECT_EQ(vector_string_interface, testInterface.vector_string_param_w_default);
    }
    {
        std::map<std::string, std::string> map_param_w_default;
        ASSERT_TRUE(nh.getParam("map_param_w_default", map_param_w_default));
        EXPECT_EQ(map_param_w_default, testInterface.map_param_w_default);
    }
    {
        int enum_int_interface;
        ASSERT_TRUE(nh.getParam("enum_int_param_w_default", enum_int_interface));
        EXPECT_EQ(enum_int_interface, testInterface.enum_int_param_w_default);
    }
    {
        std::string enum_str_interface;
        ASSERT_TRUE(nh.getParam("enum_str_param_w_default", enum_str_interface));
        EXPECT_EQ(enum_str_interface, testInterface.enum_str_param_w_default);
    }
}

TEST(RosinterfaceHandler, FromDynamicReconfigure) {
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer());
    testInterface.updater.force_update();

    ConfigType config;
    config.int_param_w_default = 2;
    config.subscriber_w_default_topic = "/in_topic";
    config.subscriber_diag_w_default_topic = "/in_point_topic";
    config.subscriber_public_w_default_topic = "/in_topic";
    config.subscriber_global_w_default_topic = "/in_topic";
    config.publisher_w_default_topic = "/out_topic";
    config.publisher_diag_w_default_topic = "/out_point_topic";
    config.publisher_public_w_default_topic = "/out_topic";
    config.publisher_global_w_default_topic = "/out_topic";
    testInterface.fromConfig(config);

    testInterface.updater.force_update();

    // params
    EXPECT_EQ(testInterface.int_param_w_default, 2);

    // subscriber
    EXPECT_EQ(testInterface.subscriber_w_default->getTopic(), "/in_topic");
    EXPECT_EQ(testInterface.subscriber_diag_w_default->getTopic(), "/in_point_topic");
    EXPECT_EQ(testInterface.subscriber_public_w_default->getTopic(), "/in_topic");
    EXPECT_EQ(testInterface.subscriber_global_w_default->getTopic(), "/in_topic");

    // publisher
    EXPECT_EQ(testInterface.publisher_w_default.getTopic(), "/out_topic");
    EXPECT_EQ(testInterface.publisher_diag_w_default.getTopic(), "/out_point_topic");
    EXPECT_EQ(testInterface.publisher_public_w_default.getTopic(), "/out_topic");
    EXPECT_EQ(testInterface.publisher_global_w_default.getTopic(), "/out_topic");
}
