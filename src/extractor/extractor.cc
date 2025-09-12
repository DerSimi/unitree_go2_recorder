#include "extractor.h"

MujocoExtractor::~MujocoExtractor()
{
}

MujocoExtractor::MujocoExtractor(mjModel *model, mjData *data, helperData *helper_data)
    : rclcpp::Node("mujoco_extractor_node"),
      mj_model_(model),
      mj_data_(data),
      helper_data_(helper_data)
{
    // Callback groups for better timing
    low_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    low_cmd_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    high_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    odometry_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    odometry_filtered_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto low_state_opt = rclcpp::SubscriptionOptions();
    low_state_opt.callback_group = low_state_group_;

    auto low_cmd_opt = rclcpp::SubscriptionOptions();
    low_cmd_opt.callback_group = low_cmd_group_;

    auto high_state_opt = rclcpp::SubscriptionOptions();
    high_state_opt.callback_group = high_state_group_;

    auto odom_opt = rclcpp::SubscriptionOptions();
    odom_opt.callback_group = odometry_group_;

    auto odom_filtered_opt = rclcpp::SubscriptionOptions();
    odom_filtered_opt.callback_group = odometry_filtered_group_;

    // Subscribe topics
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        TOPIC_LOWSTATE, rclcpp::SensorDataQoS(),
        std::bind(&MujocoExtractor::lowStateCallback, this, std::placeholders::_1), low_state_opt);

    low_cmd_sub_ = this->create_subscription<unitree_go::msg::LowCmd>(
        TOPIC_LOWCMD, rclcpp::SensorDataQoS(),
        std::bind(&MujocoExtractor::lowCmdCallback, this, std::placeholders::_1), low_cmd_opt);

    if (USE_ODOMETRY)
    {
        odometry_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            TOPIC_ODOMETRY, rclcpp::SensorDataQoS(),
            std::bind(&MujocoExtractor::odometryCallback, this, std::placeholders::_1), odom_opt);

        odometry_filtered_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            TOPIC_ODOMETRY_FILTERED, rclcpp::SensorDataQoS(),
            std::bind(&MujocoExtractor::odometryFilteredCallback, this, std::placeholders::_1), odom_filtered_opt);
    }
    else
    {
        high_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            TOPIC_HIGHSTATE, rclcpp::SensorDataQoS(),
            std::bind(&MujocoExtractor::highStateCallback, this, std::placeholders::_1), high_state_opt);
    }

    checkSensor();
}

/*
nu ctrl dimension
nq qpos dimension
nv qvel dimension

12 Motoren, also ctrl = 12

qpos dim ist 19:
7 für basis: position, quat
wir haben 12 gelenke, pro gelenk ein winkel, 7 + 12 = 19

nv qvel dimension ist 18:
6 für basis: linear velocity, angular velocity
wir haben 12 gelenke, pro gelenk eine Geschwindigkeit, 6 + 12 = 18
*/

void MujocoExtractor::lowCmdCallback(const unitree_go::msg::LowCmd::SharedPtr cmd)
{
    // std::cout << "LowCmd received" << std::endl;

    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    if (mj_data_)
    {
        LowCmdData data(num_motor_);
        data.timestamp = timestamp;

        for (int i = 0; i < num_motor_; i++)
        {
            data.ctrl[i] = cmd->motor_cmd[i].tau +
                           cmd->motor_cmd[i].kp * (cmd->motor_cmd[i].q - mj_data_->sensordata[i]) +
                           cmd->motor_cmd[i].kd * (cmd->motor_cmd[i].dq - mj_data_->sensordata[i + num_motor_]);

            // Save additional data
            data.q[i] = cmd->motor_cmd[i].q;
            data.dq[i] = cmd->motor_cmd[i].dq;
            data.tau[i] = cmd->motor_cmd[i].tau;
            data.kp[i] = cmd->motor_cmd[i].kp;
            data.kd[i] = cmd->motor_cmd[i].kd;
        }

        // Lock mutex and write into buffer
        {
            std::lock_guard<std::mutex> lock(mtx_);
            low_cmd_buffer_.push_back(data);

            if (low_cmd_buffer_.size() > SYNC_BUFFER_MAX_SIZE)
            {
                low_cmd_buffer_.pop_front();
            }
        } // Free mutex
    }
}

void MujocoExtractor::lowStateCallback(const unitree_go::msg::LowState::SharedPtr state)
{
    // std::cout << "LowState received" << std::endl;
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    if (mj_data_)
    {
        LowStateData data(num_motor_);
        data.timestamp = timestamp;

        // Inject IMU state
        if (have_frame_sensor_)
        {
            // Rotation quaternion
            data.base_quat[0] = state->imu_state.quaternion[0];
            data.base_quat[1] = state->imu_state.quaternion[1];
            data.base_quat[2] = state->imu_state.quaternion[2];
            data.base_quat[3] = state->imu_state.quaternion[3];

            // Angular velocity
            data.base_ang_vel[0] = state->imu_state.gyroscope[0];
            data.base_ang_vel[1] = state->imu_state.gyroscope[1];
            data.base_ang_vel[2] = state->imu_state.gyroscope[2];

            // Linear velocity is not in here, only acceleration!!!
        }

        // Inject motor state
        for (int i = 0; i < num_motor_; i++)
        {
            // angle
            data.qpos_joints[i] = state->motor_state[i].q;

            // angular velocity
            data.qvel_joints[i] = state->motor_state[i].dq;

            // more information to collect
            data.tau_est[i] = state->motor_state[i].tau_est;
            data.q_raw[i] = state->motor_state[i].q_raw;
            data.dq_raw[i] = state->motor_state[i].dq_raw;
        }

        // Lock mutex and write into buffer
        {
            std::lock_guard<std::mutex> lock(mtx_);
            low_state_buffer_.push_back(data);

            if (low_state_buffer_.size() > SYNC_BUFFER_MAX_SIZE)
            {
                low_state_buffer_.pop_front();
            }
        } // Free mutex
    }
}

void MujocoExtractor::highStateCallback(const unitree_go::msg::SportModeState::SharedPtr state)
{
    // std::cout << "HighState received" << std::endl;

    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Inject position and velocity into mujoco data
    if (mj_data_ && have_frame_sensor_)
    {
        HighStateData data;
        data.timestamp = timestamp;

        // position
        data.base_pos[0] = state->position[0];
        data.base_pos[1] = state->position[1];
        data.base_pos[2] = state->position[2];

        // velocity
        data.base_lin_vel[0] = state->velocity[0];
        data.base_lin_vel[1] = state->velocity[1];
        data.base_lin_vel[2] = state->velocity[2];

        // Lock mutex and write into buffer
        {
            std::lock_guard<std::mutex> lock(mtx_);
            high_state_buffer_.push_back(data);

            if (high_state_buffer_.size() > SYNC_BUFFER_MAX_SIZE)
            {
                high_state_buffer_.pop_front();
            }
        } // Free mutex
    }
}

void MujocoExtractor::odometryCallback(const tf2_msgs::msg::TFMessage::SharedPtr state)
{
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Inject position and velocity into mujoco data
    if (mj_data_ && have_frame_sensor_)
    {
        OdometryData data;
        data.timestamp = timestamp;

        // position
        data.base_pos[0] = state->transforms[0].transform.translation.x;
        data.base_pos[1] = state->transforms[0].transform.translation.y;
        data.base_pos[2] = state->transforms[0].transform.translation.z;

        // Lock mutex and write into buffer
        {
            std::lock_guard<std::mutex> lock(mtx_);
            odometry_buffer_.push_back(data);

            if (odometry_buffer_.size() > SYNC_BUFFER_MAX_SIZE)
            {
                odometry_buffer_.pop_front();
            }
        } // Free mutex
    }
}

void MujocoExtractor::odometryFilteredCallback(const nav_msgs::msg::Odometry::SharedPtr state)
{
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Inject position and velocity into mujoco data
    if (mj_data_ && have_frame_sensor_)
    {
        OdometryFilteredData data;
        data.timestamp = timestamp;
        data.base_lin_vel[0] = state->twist.twist.linear.x;
        data.base_lin_vel[1] = state->twist.twist.linear.y;
        data.base_lin_vel[2] = state->twist.twist.linear.z;

        // Lock mutex and write into buffer
        {
            std::lock_guard<std::mutex> lock(mtx_);
            odometry_filtered_buffer_.push_back(data);

            if (odometry_filtered_buffer_.size() > SYNC_BUFFER_MAX_SIZE)
            {
                odometry_filtered_buffer_.pop_front();
            }
        } // Free mutex
    }
}

void MujocoExtractor::insertSynchronizedData(const LowCmdData &cmd, const LowStateData &low_state, const HighStateData &high_state)
{
    // Base position and orientation
    mj_data_->qpos[0] = high_state.base_pos[0];
    mj_data_->qpos[1] = high_state.base_pos[1];
    mj_data_->qpos[2] = high_state.base_pos[2];

    mj_data_->qpos[3] = low_state.base_quat[0];
    mj_data_->qpos[4] = low_state.base_quat[1];
    mj_data_->qpos[5] = low_state.base_quat[2];
    mj_data_->qpos[6] = low_state.base_quat[3];

    // Base linear and angular velocity
    mj_data_->qvel[0] = high_state.base_lin_vel[0];
    mj_data_->qvel[1] = high_state.base_lin_vel[1];
    mj_data_->qvel[2] = high_state.base_lin_vel[2];

    mj_data_->qvel[3] = low_state.base_ang_vel[0];
    mj_data_->qvel[4] = low_state.base_ang_vel[1];
    mj_data_->qvel[5] = low_state.base_ang_vel[2];

    // Motor angle, velocity and control
    for (int i = 0; i < num_motor_; i++)
    {
        mj_data_->qpos[7 + i] = low_state.qpos_joints[i];
        mj_data_->qvel[6 + i] = low_state.qvel_joints[i];
        mj_data_->ctrl[i] = cmd.ctrl[i];
    }

    // Write back helper data
    for (int i = 0; i < num_motor_; i++)
    {
        // Low command
        helper_data_->q[i] = cmd.q[i];
        helper_data_->dq[i] = cmd.dq[i];
        helper_data_->tau[i] = cmd.tau[i];
        helper_data_->kp[i] = cmd.kp[i];
        helper_data_->kd[i] = cmd.kd[i];

        // Low state
        helper_data_->tau_est[i] = low_state.tau_est[i];
        helper_data_->q_raw[i] = low_state.q_raw[i];
        helper_data_->dq_raw[i] = low_state.dq_raw[i];
    }
}

// Synchronization logic: First choose the oldest low cmd entry, and then choose the corresponding high
// and low state entry which is closed to it, but NOT newer!
bool MujocoExtractor::GetSynchronizedState(double &out_timestamp, bool disabled)
{
    if (!mj_data_)
        return false;

    std::lock_guard<std::mutex> lock(mtx_);

    // We use low cmd buffer as base, as it contains the most data
    if (USE_ODOMETRY && (odometry_buffer_.empty() || odometry_filtered_buffer_.empty()))
    {
        return false;
    }

    if (!USE_ODOMETRY && high_state_buffer_.empty())
    {
        return false;
    }

    if (low_cmd_buffer_.empty() || low_state_buffer_.empty())
    {
        return false;
    }

    // Easy logic to disable synchronization
    if (disabled)
    {
        const auto &ref_low_cmd = low_cmd_buffer_.back();
        const auto &match_low_state = low_state_buffer_.back();

        HighStateData match_high_state;

        if (!USE_ODOMETRY)
        {
            match_high_state = high_state_buffer_.back();
        }
        else
        {
            OdometryData odom = odometry_buffer_.back();
            OdometryFilteredData odom_filt = odometry_filtered_buffer_.back();

            match_high_state.timestamp = ref_low_cmd.timestamp;
            match_high_state.base_pos[0] = odom.base_pos[0];
            match_high_state.base_pos[1] = odom.base_pos[1];
            match_high_state.base_pos[2] = odom.base_pos[2];

            match_high_state.base_lin_vel[0] = odom_filt.base_lin_vel[0];
            match_high_state.base_lin_vel[1] = odom_filt.base_lin_vel[1];
            match_high_state.base_lin_vel[2] = odom_filt.base_lin_vel[2];
        }

        insertSynchronizedData(ref_low_cmd, match_low_state, match_high_state);

        low_cmd_buffer_.clear();
        high_state_buffer_.clear();
        low_state_buffer_.clear();
        odometry_buffer_.clear();
        odometry_filtered_buffer_.clear();
        return true;
    }

    const auto &ref_low_cmd = low_cmd_buffer_.front();
    double T_ref = ref_low_cmd.timestamp;

    // Get the low state first
    auto it_low = low_state_buffer_.rbegin();
    while (it_low != low_state_buffer_.rend() && it_low->timestamp > T_ref)
    {
        ++it_low;
    }

    if (it_low == low_state_buffer_.rend())
    {
        low_cmd_buffer_.pop_front();
        return false;
    }

    const LowStateData *match_low_state = &(*it_low);

    // Now get the high state
    HighStateData high_state_data;

    if (!USE_ODOMETRY)
    {
        auto it_high = high_state_buffer_.rbegin();
        while (it_high != high_state_buffer_.rend() && it_high->timestamp > T_ref)
        {
            ++it_high;
        }

        if (it_high == high_state_buffer_.rend())
        {
            low_cmd_buffer_.pop_front();
            return false;
        }

        high_state_data = *it_high;

        high_state_buffer_.erase(high_state_buffer_.begin(), it_high.base() - 1);
    }
    else // Use odometry data
    {
        auto it_odometry = odometry_buffer_.rbegin();
        while (it_odometry != odometry_buffer_.rend() && it_odometry->timestamp > T_ref)
        {
            ++it_odometry;
        }

        auto it_odometry_filtered = odometry_filtered_buffer_.rbegin();
        while (it_odometry_filtered != odometry_filtered_buffer_.rend() && it_odometry_filtered->timestamp > T_ref)
        {
            ++it_odometry_filtered;
        }

        if (it_odometry == odometry_buffer_.rend() || it_odometry_filtered == odometry_filtered_buffer_.rend())
        {
            low_cmd_buffer_.pop_front();
            return false;
        }

        OdometryData odom = *it_odometry;
        OdometryFilteredData odom_filt = *it_odometry_filtered;

        high_state_data.timestamp = T_ref;

        high_state_data.base_pos[0] = odom.base_pos[0];
        high_state_data.base_pos[1] = odom.base_pos[1];
        high_state_data.base_pos[2] = odom.base_pos[2];

        high_state_data.base_lin_vel[0] = odom_filt.base_lin_vel[0];
        high_state_data.base_lin_vel[1] = odom_filt.base_lin_vel[1];
        high_state_data.base_lin_vel[2] = odom_filt.base_lin_vel[2];

        odometry_buffer_.erase(odometry_buffer_.begin(), it_odometry.base() - 1);
        odometry_filtered_buffer_.erase(odometry_filtered_buffer_.begin(), it_odometry_filtered.base() - 1);
    }

    // Insert data into mj_data
    insertSynchronizedData(ref_low_cmd, *match_low_state, high_state_data);

    out_timestamp = T_ref;
    low_cmd_buffer_.pop_front();

    low_state_buffer_.erase(low_state_buffer_.begin(), it_low.base() - 1);

    return true;
}

void MujocoExtractor::checkSensor()
{
    num_motor_ = mj_model_->nu;
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    cout << "num motors: " << num_motor_ << endl;
    cout << "qpos dim: " << mj_model_->nq << endl;
    cout << "qvel dim: " << mj_model_->nv << endl;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }
}