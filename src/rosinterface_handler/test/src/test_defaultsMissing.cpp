#include <gtest/gtest.h>
#include <rosinterface_handler/DefaultsMissingInterface.h>

typedef rosinterface_handler::DefaultsMissingInterface IfType;
typedef rosinterface_handler::DefaultsMissingConfig ConfigType;

TEST(RosinterfaceHandler, DefaultsMissing) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_THROW(testInterface.fromParamServer(), std::runtime_error);
}
