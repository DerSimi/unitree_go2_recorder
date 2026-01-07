#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>

#include "unitree_go/msg/low_cmd.hpp"
#include "timed_topics/msg/timed_low_cmd.hpp"

#include "base/data_source.hpp"

#define TOPIC_LOWCMD "/timedlowcmd"

struct LowCmdData
{
    rclcpp::Time stamp;
    std::vector<mjtNum> ctrl;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
    std::vector<float> kp;
    std::vector<float> kd;

    LowCmdData(size_t num_motor)
        : ctrl(num_motor),
          q(num_motor),
          dq(num_motor),
          tau(num_motor),
          kp(num_motor),
          kd(num_motor)
    {
    }
};

class LowCmdSource : public DataSource<LowCmdData>
{
public:
    LowCmdSource(mjData *data) : data_(data) {};
    void subscribe(rclcpp::Node *node) override;
    std::deque<LowCmdData> &buffer() override { return buffer_; }

private:
    void callback(const timed_topics::msg::TimedLowCmd::SharedPtr msg);
    
    rclcpp::Subscription<timed_topics::msg::TimedLowCmd>::SharedPtr topic_sub_;
    std::deque<LowCmdData> buffer_;
    mjData *data_;
    
public:
    bool get_closest_match(rclcpp::Time &time, void *res);

};