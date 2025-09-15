#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>
#include <Eigen/Geometry>
#include "nav_msgs/msg/odometry.hpp"

#include "base/data_source.hpp"

#define TOPIC_ODOMETRY_FILTERED "/odometry/filtered"
// Odometry z offset, change this to your setup
#define ODOMETRY_Z_OFFSET 0.32

struct OdometryData
{
    double timestamp;
    mjtNum base_pos[3];     // base position
    mjtNum base_lin_vel[3]; // base linear velocity
    mjtNum base_quat[4];
    mjtNum base_ang_vel[3];
};

class OdometrySource : public DataSource<OdometryData>
{
public:
    void subscribe(rclcpp::Node *node) override;
    std::deque<OdometryData> &buffer() override { return buffer_; }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr topic_sub_;
    std::deque<OdometryData> buffer_;

    bool initial_rotation_captured_ = false;
    Eigen::Isometry3d world_to_odom_correction_;
};