#include "base/low_state/low_state.hpp"

void LowStateSource::subscribe(rclcpp::Node *node)
{
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    topic_sub_ = node->create_subscription<timed_topics::msg::TimedLowState>(
        TOPIC_LOWSTATE, rclcpp::SensorDataQoS(),
        std::bind(&LowStateSource::callback, this, std::placeholders::_1), sub_opt);
}

void LowStateSource::callback(const timed_topics::msg::TimedLowState::SharedPtr msg)
{
    const unitree_go::msg::LowState &state = msg->state;

    LowStateData data(NUM_MOTOR);
    data.stamp = rclcpp::Time(msg->stamp);

    // Rotation
    data.base_quat[0] = state.imu_state.quaternion[0]; // w
    data.base_quat[1] = state.imu_state.quaternion[1]; // x
    data.base_quat[2] = state.imu_state.quaternion[2]; // y
    data.base_quat[3] = state.imu_state.quaternion[3]; // z

    // Angular velocity
    data.base_ang_vel[0] = state.imu_state.gyroscope[0];
    data.base_ang_vel[1] = state.imu_state.gyroscope[1];
    data.base_ang_vel[2] = state.imu_state.gyroscope[2];

    // Inject motor state
    for (int i = 0; i < NUM_MOTOR; i++)
    {
        // angle
        data.qpos_joints[i] = state.motor_state[i].q;

        // angular velocity
        data.qvel_joints[i] = state.motor_state[i].dq;

        // more information to collect
        data.tau_est[i] = state.motor_state[i].tau_est;
        data.q_raw[i] = state.motor_state[i].q_raw;
        data.dq_raw[i] = state.motor_state[i].dq_raw;
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

bool LowStateSource::get_closest_match(rclcpp::Time &time, LowStateData *res)
{
    NOT_IMPLEMENTED;
}