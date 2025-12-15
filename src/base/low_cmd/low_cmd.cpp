#include "base/low_cmd/low_cmd.hpp"

void LowCmdSource::subscribe(rclcpp::Node *node)
{
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    topic_sub_ = node->create_subscription<timed_topics::msg::TimedLowCmd>(
        TOPIC_LOWCMD, rclcpp::SensorDataQoS(),
        std::bind(&LowCmdSource::callback, this, std::placeholders::_1), sub_opt);
}

void LowCmdSource::callback(const timed_topics::msg::TimedLowCmd::SharedPtr msg)
{
    const unitree_go::msg::LowCmd &state = msg->state;

    LowCmdData data(NUM_MOTOR);
    data.stamp = rclcpp::Time(msg->stamp);

    for (int i = 0; i < NUM_MOTOR; i++)
    {
        data.ctrl[i] = state.motor_cmd[i].tau +
                       state.motor_cmd[i].kp * (state.motor_cmd[i].q - data_->sensordata[i]) +
                       state.motor_cmd[i].kd * (state.motor_cmd[i].dq - data_->sensordata[i + NUM_MOTOR]);

        // Save additional data
        data.q[i] = state.motor_cmd[i].q;
        data.dq[i] = state.motor_cmd[i].dq;
        data.tau[i] = state.motor_cmd[i].tau;
        data.kp[i] = state.motor_cmd[i].kp;
        data.kd[i] = state.motor_cmd[i].kd;
    }

    // Lock mutex and write into buffer
    {
        std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);
        buffer_.push_back(data);

        if (buffer_.size() > SYNC_BUFFER_MAX_SIZE)
        {
            buffer_.pop_front();
        }
    } // Free mutex
}

bool LowCmdSource::get_closest_match(rclcpp::Time &time, LowCmdData *res)
{
    if (buffer_.empty())
        return false;

    auto closest_it = buffer_.begin();
    auto min_dt = std::abs((closest_it->stamp - time).nanoseconds());

    for (auto it = buffer_.begin(); it != buffer_.end(); ++it)
    {
        auto dt = std::abs((it->stamp - time).nanoseconds());
        if (dt < min_dt)
        {
            min_dt = dt;
            closest_it = it;
        }
    }

    *res = *closest_it;
    return true;
}