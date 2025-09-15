#include "base/low_cmd/low_cmd.hpp"

void LowCmdSource::subscribe(rclcpp::Node *node)
{
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    topic_sub_ = node->create_subscription<unitree_go::msg::LowCmd>(
        TOPIC_LOWCMD, rclcpp::SensorDataQoS(),
        std::bind(&LowCmdSource::callback, this, std::placeholders::_1), sub_opt);
}

void LowCmdSource::callback(const unitree_go::msg::LowCmd::SharedPtr msg)
{
    LowCmdData data(NUM_MOTOR);
    data.timestamp = get_timestamp();

    for (int i = 0; i < NUM_MOTOR; i++)
    {
        data.ctrl[i] = msg->motor_cmd[i].tau +
                       msg->motor_cmd[i].kp * (msg->motor_cmd[i].q - data_->sensordata[i]) +
                       msg->motor_cmd[i].kd * (msg->motor_cmd[i].dq - data_->sensordata[i + NUM_MOTOR]);

        // Save additional data
        data.q[i] = msg->motor_cmd[i].q;
        data.dq[i] = msg->motor_cmd[i].dq;
        data.tau[i] = msg->motor_cmd[i].tau;
        data.kp[i] = msg->motor_cmd[i].kp;
        data.kd[i] = msg->motor_cmd[i].kd;
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