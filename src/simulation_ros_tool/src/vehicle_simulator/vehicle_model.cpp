#include "vehicle_model.h"
#include <limits>

VehicleModel::VehicleModel(const Eigen::Affine3d& initial_pose) : pose(initial_pose) {
}

void VehicleModel::setVelocity(double velocity) {
    velocity_ = velocity;
}

void VehicleModel::setSteeringAngle(double steering_angle) {
    steering_angle_ = steering_angle;
}

void VehicleModel::setWheelBase(double wheel_base) {
    wheel_base_ = wheel_base;
}

Eigen::Affine3d VehicleModel::step(const double time_step) {
    using namespace Eigen;

    const double yaw_rate = (velocity_ / wheel_base_) * std::tan(steering_angle_);

    const Translation3d delta_translation(velocity_ * time_step, 0.0, 0.0);
    const AngleAxisd delta_rotation(yaw_rate * time_step, Vector3d::UnitZ());

    pose *= delta_translation;
    pose *= delta_rotation;

    return pose; // rear axle
}
