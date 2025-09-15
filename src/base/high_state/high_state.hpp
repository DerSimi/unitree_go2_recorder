#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>

#include "unitree_go/msg/sport_mode_state.hpp"

#include "base/data_source.hpp"

#define TOPIC_HIGHSTATE "/sportmodestate"

struct HighStateData
{
    double timestamp;
    mjtNum base_pos[3];     // base position
    mjtNum base_lin_vel[3]; // base linear velocity
};

class HighStateSource : public DataSource<HighStateData>
{
public:
    void subscribe(rclcpp::Node *node) override;
    std::deque<HighStateData> &buffer() override { return buffer_; }

private:
    void callback(const unitree_go::msg::SportModeState::SharedPtr msg);

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr topic_sub_;
    std::deque<HighStateData> buffer_;
};