#pragma once

#include <chrono>
#include <vector>
#include <mujoco/mujoco.h>

#include "unitree_go/msg/low_cmd.hpp"

#include "base/data_source.hpp"

#define TOPIC_LOWCMD "/lowcmd"

struct LowCmdData
{
    double timestamp;
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
    void callback(const unitree_go::msg::LowCmd::SharedPtr msg);

    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr topic_sub_;
    std::deque<LowCmdData> buffer_;
    mjData *data_;
};