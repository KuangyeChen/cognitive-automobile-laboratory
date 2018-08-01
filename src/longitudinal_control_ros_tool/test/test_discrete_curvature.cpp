#include <gtest/gtest.h>
#include "lateral_control_ros_tool/discrete_curvature.h"

using namespace lateral_control_ros_tool;

TEST(DescreteCurvature, simple_circle) {

 const Eigen::Vector2d p3(0.0, 1.0);
 const Eigen::Vector2d p2 = Eigen::Vector2d(0.1, 1.0).normalized();
 const Eigen::Vector2d p1 = Eigen::Vector2d(0.2, 1.0).normalized();

 EXPECT_NEAR(discreteCurvature(p1, p2, p3), 1.0, 0.0001);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
