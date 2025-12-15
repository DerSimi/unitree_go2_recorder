#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>

#include "unitree_go/msg/low_state.hpp"
#include "timed_topics/msg/timed_low_state.hpp"

#include "base/data_source.hpp"

#define TOPIC_LOWSTATE "/timed_lowstate"

struct LowStateData
{
    rclcpp::Time stamp;
    std::vector<mjtNum> qpos_joints; // 12 joint angles
    std::vector<mjtNum> qvel_joints; // 12 joint velocities
    std::vector<float> tau_est;      // 12 estimated motor torques
    std::vector<float> q_raw;        // 12 raw joint positions
    std::vector<float> dq_raw;       // 12 raw joint velocities
    mjtNum base_quat[4];             // base rotation
    mjtNum base_ang_vel[3];          // base angular velocity

    LowStateData(size_t num_motor)
        : qpos_joints(num_motor),
          qvel_joints(num_motor),
          tau_est(num_motor),
          q_raw(num_motor),
          dq_raw(num_motor)
    {
    }
};

class LowStateSource : public DataSource<LowStateData>
{
public:
    void subscribe(rclcpp::Node *node) override;
    std::deque<LowStateData> &buffer() override { return buffer_; }

private:
    void callback(const timed_topics::msg::TimedLowState::SharedPtr msg);

    rclcpp::Subscription<timed_topics::msg::TimedLowState>::SharedPtr topic_sub_;
    std::deque<LowStateData> buffer_;

public:
    bool get_closest_match(rclcpp::Time &time, LowStateData *res);
};