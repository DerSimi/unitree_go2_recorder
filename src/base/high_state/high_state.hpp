#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>

#include "unitree_go/msg/sport_mode_state.hpp"
#include "timed_topics/msg/timed_sport_mode_state.hpp"

#include "base/data_source.hpp"

#define TOPIC_HIGHSTATE "/timed_sportmodestate"

struct HighStateData
{
    rclcpp::Time stamp;
    mjtNum base_pos[3];     // base position
    mjtNum base_lin_vel[3]; // base linear velocity
};

class HighStateSource : public DataSource<HighStateData>
{
private:
    void callback(const timed_topics::msg::TimedSportModeState::SharedPtr msg);

    rclcpp::Subscription<timed_topics::msg::TimedSportModeState>::SharedPtr topic_sub_;
    std::deque<HighStateData> buffer_;

public:
    void subscribe(rclcpp::Node *node) override;
    std::deque<HighStateData> &buffer() override { return buffer_; }
    bool get_closest_match(rclcpp::Time &time, HighStateData *res);
};