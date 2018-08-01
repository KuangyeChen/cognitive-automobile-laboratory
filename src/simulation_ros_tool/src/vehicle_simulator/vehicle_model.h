#pragma once

#include <Eigen/Dense>

class VehicleModel {
public:
    VehicleModel() = default;
    VehicleModel(const Eigen::Affine3d& initial_pose);
    void setVelocity(double velocity);
    void setSteeringAngle(double steering_angle);
    void setWheelBase(double wheel_base);
    Eigen::Affine3d step(double time_step);

private:
    double wheel_base_ = 0.5;
    double velocity_ = 0.0;
    double steering_angle_ = 0.0;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity(); // rear axle
};
