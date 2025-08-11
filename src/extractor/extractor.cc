#include "extractor.h"

MujocoExtractor::~MujocoExtractor()
{
}

MujocoExtractor::MujocoExtractor(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();
    low_cmd_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_suber_->InitChannel(bind(&MujocoExtractor::LowCmdHandler, this, placeholders::_1), 1);

    low_state_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_suber_->InitChannel(bind(&MujocoExtractor::LowStateHandler, this, placeholders::_1), 1);

    high_state_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_state_suber_->InitChannel(bind(&MujocoExtractor::HighStateHandler, this, placeholders::_1), 1);
}

void MujocoExtractor::LowCmdHandler(const void *msg)
{
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        LowCmdData data;
        data.timestamp = timestamp;
        data.ctrl.resize(num_motor_);

        for (int i = 0; i < num_motor_; i++)
        {
            data.ctrl[i] = cmd->motor_cmd()[i].tau() +
                           cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                           cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
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

void MujocoExtractor::LowStateHandler(const void *msg)
{
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    const unitree_go::msg::dds_::LowState_ *state = (const unitree_go::msg::dds_::LowState_ *)msg;

    if (mj_data_)
    {
        LowStateData data;
        data.timestamp = timestamp;

        // Inject IMU state
        if (have_frame_sensor_)
        {
            // Rotation quaternion
            data.base_quat[0] = state->imu_state().quaternion()[0];
            data.base_quat[1] = state->imu_state().quaternion()[1];
            data.base_quat[2] = state->imu_state().quaternion()[2];
            data.base_quat[3] = state->imu_state().quaternion()[3];

            // Angular velocity
            data.base_ang_vel[0] = state->imu_state().gyroscope()[0];
            data.base_ang_vel[1] = state->imu_state().gyroscope()[1];
            data.base_ang_vel[2] = state->imu_state().gyroscope()[2];

            // Linear velocity is not in here, only acceleration!!!
        }

        data.qpos_joints.resize(num_motor_);
        data.qvel_joints.resize(num_motor_);

        // Inject motor state
        for (int i = 0; i < num_motor_; i++)
        {
            // angle
            data.qpos_joints[i] = state->motor_state()[i].q();

            // angular velocity
            data.qvel_joints[i] = state->motor_state()[i].dq();
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

void MujocoExtractor::HighStateHandler(const void *msg)
{
    auto now = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    const unitree_go::msg::dds_::SportModeState_ *state = (const unitree_go::msg::dds_::SportModeState_ *)msg;

    // Inject position and velocity into mujoco data
    if (mj_data_ && have_frame_sensor_)
    {
        HighStateData data;
        data.timestamp = timestamp;

        // position
        data.base_pos[0] = state->position()[0];
        data.base_pos[1] = state->position()[1];
        data.base_pos[2] = state->position()[2];

        // velocity
        data.base_lin_vel[0] = state->velocity()[0];
        data.base_lin_vel[1] = state->velocity()[1];
        data.base_lin_vel[2] = state->velocity()[2];

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

// Synchronization logic: First choose the oldest low cmd entry, and then choose the corresponding high
// and low state entry which is closed to it, but NOT newer!
bool MujocoExtractor::GetSynchronizedState(double &out_timestamp)
{
    if (!mj_data_)
        return false;

    std::lock_guard<std::mutex> lock(mtx_);

    // We use low cmd buffer as base, as it contains the most data
    if (low_cmd_buffer_.empty())
    {
        return false;
    }

    const auto &ref_low_cmd = low_cmd_buffer_.front();
    double T_ref = ref_low_cmd.timestamp;

    auto it_high = high_state_buffer_.rbegin();
    while (it_high != high_state_buffer_.rend() && it_high->timestamp > T_ref)
    {
        ++it_high;
    }

    auto it_low = low_state_buffer_.rbegin();
    while (it_low != low_state_buffer_.rend() && it_low->timestamp > T_ref)
    {
        ++it_low;
    }

    if (it_high == high_state_buffer_.rend() || it_low == low_state_buffer_.rend())
    {
        low_cmd_buffer_.pop_front();
        return false;
    }

    const auto &match_high_state = *it_high;
    const auto &match_low_state = *it_low;

    // Insert data into mj_data

    // Base position and orientation
    mj_data_->qpos[0] = match_high_state.base_pos[0];
    mj_data_->qpos[1] = match_high_state.base_pos[1];
    mj_data_->qpos[2] = match_high_state.base_pos[2];

    mj_data_->qpos[3] = match_low_state.base_quat[0];
    mj_data_->qpos[4] = match_low_state.base_quat[1];
    mj_data_->qpos[5] = match_low_state.base_quat[2];
    mj_data_->qpos[6] = match_low_state.base_quat[3];

    // Base linear and angular velocity
    mj_data_->qvel[0] = match_high_state.base_lin_vel[0];
    mj_data_->qvel[1] = match_high_state.base_lin_vel[1];
    mj_data_->qvel[2] = match_high_state.base_lin_vel[2];

    mj_data_->qvel[3] = match_low_state.base_ang_vel[0];
    mj_data_->qvel[4] = match_low_state.base_ang_vel[1];
    mj_data_->qvel[5] = match_low_state.base_ang_vel[2];

    // Motor angle, velocity and control
    for (int i = 0; i < num_motor_; i++)
    {
        mj_data_->qpos[7 + i] = match_low_state.qpos_joints[i];
        mj_data_->qvel[6 + i] = match_low_state.qvel_joints[i];
        mj_data_->ctrl[i] = ref_low_cmd.ctrl[i];
    }

    out_timestamp = T_ref;
    low_cmd_buffer_.pop_front();

    high_state_buffer_.erase(high_state_buffer_.begin(), it_high.base() - 1);
    low_state_buffer_.erase(low_state_buffer_.begin(), it_low.base() - 1);

    return true;
}

void MujocoExtractor::Run()
{
    while (1)
    {
        sleep(2);
    }
}

void MujocoExtractor::CheckSensor()
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