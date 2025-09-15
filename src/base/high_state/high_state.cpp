#include "base/high_state/high_state.hpp"

void HighStateSource::subscribe(rclcpp::Node *node)
{
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    topic_sub_ = node->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, rclcpp::SensorDataQoS(),
        std::bind(&HighStateSource::callback, this, std::placeholders::_1), sub_opt);
}

void HighStateSource::callback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
    HighStateData data;
    data.timestamp = get_timestamp();

    // position
    data.base_pos[0] = msg->position[0];
    data.base_pos[1] = msg->position[1];
    data.base_pos[2] = msg->position[2];

    // velocity
    data.base_lin_vel[0] = msg->velocity[0];
    data.base_lin_vel[1] = msg->velocity[1];
    data.base_lin_vel[2] = msg->velocity[2];

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