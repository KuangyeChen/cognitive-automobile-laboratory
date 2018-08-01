#include <gtest/gtest.h>
#include <rosinterface_handler/MinMaxInterface.h>

typedef rosinterface_handler::MinMaxInterface IfType;
typedef rosinterface_handler::MinMaxConfig ConfigType;

TEST(RosinterfaceHandler, MinMax) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer());

    ASSERT_EQ(2, testInterface.int_param_w_minmax);
    ASSERT_DOUBLE_EQ(2., testInterface.double_param_w_minmax);

    ASSERT_EQ(std::vector<int>({0, 2, 2}), testInterface.vector_int_param_w_minmax);
    ASSERT_EQ(std::vector<double>({0., 1.2, 2.}), testInterface.vector_double_param_w_minmax);

    std::map<std::string, double> tmp{{"value1", 0.}, {"value2", 1.2}, {"value3", 2.}};
    ASSERT_EQ(tmp, testInterface.map_param_w_minmax);
}
