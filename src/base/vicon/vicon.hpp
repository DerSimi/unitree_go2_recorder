#pragma once

#include <array>
#include <iostream>
#include <chrono>
#include <vector>
#include <unistd.h>
#include <thread>

#include <mujoco/mujoco.h>

#include <vicon-datastream-sdk/DataStreamClient.h>

#include <rclcpp/rclcpp.hpp>

#include "common.hpp"
#include "base/data_source.hpp"

struct ViconData
{
    rclcpp::Time stamp;
    mjtNum base_pos[3];     // base position
    mjtNum orientation[4];  // base orientation
    mjtNum world_lin_vel[3]; // base linear velocity
};

class ViconDataSource : public DataSource<ViconData>
{
private:
    void callback(const std::array<double, 3> &position, const std::array<double, 4> &orientation);

    std::string hostname_;
    std::string subject_name_;
    ViconDataStreamSDK::CPP::Client client_;

    std::thread thread_;
    bool running_ = false;

    // For velocity estimation
    // bool hast_last_position_ = false;
    // double last_timestamp_ = 0.0;
    // std::array<double, 3> last_position_;

    std::deque<ViconData> buffer_;

public:
    explicit ViconDataSource(const std::string &hostname, const std::string &subject_name);
    ~ViconDataSource() override;
    void subscribe(rclcpp::Node *node) override;
    std::deque<ViconData> &buffer() override { return buffer_; }
    // Returns the interpolated step:
    // Searches for the two vicon steps between the stamp, and interpolates the position change in that timeframe.
    bool get_closest_match(rclcpp::Time &time, void *res);
};