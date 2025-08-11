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
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
            data.ctrl[i] = mj_data_->ctrl[i];
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
            mj_data_->qpos[3] = state->imu_state().quaternion()[0];
            mj_data_->qpos[4] = state->imu_state().quaternion()[1];
            mj_data_->qpos[5] = state->imu_state().quaternion()[2];
            mj_data_->qpos[6] = state->imu_state().quaternion()[3];

            data.base_quat[0] = mj_data_->qpos[3];
            data.base_quat[1] = mj_data_->qpos[4];
            data.base_quat[2] = mj_data_->qpos[5];
            data.base_quat[3] = mj_data_->qpos[6];

            // Angular velocity
            mj_data_->qvel[0] = state->imu_state().gyroscope()[0];
            mj_data_->qvel[1] = state->imu_state().gyroscope()[1];
            mj_data_->qvel[2] = state->imu_state().gyroscope()[2];

            data.base_ang_vel[0] = mj_data_->qvel[0];
            data.base_ang_vel[1] = mj_data_->qvel[1];
            data.base_ang_vel[2] = mj_data_->qvel[2];

            // Linear velocity is not in here, only acceleration!!!
        }

        data.qpos_joints.resize(num_motor_);
        data.qvel_joints.resize(num_motor_);

        // Inject motor state
        for (int i = 0; i < num_motor_; i++)
        {
            // angle
            mj_data_->qpos[7 + i] = state->motor_state()[i].q();
            data.qpos_joints[i] = mj_data_->qpos[7 + i];

            // angular velocity
            mj_data_->qvel[7 + i] = state->motor_state()[i].dq();
            data.qvel_joints[i] = mj_data_->qvel[7 + i];
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
        mj_data_->qpos[0] = state->position()[0];
        mj_data_->qpos[1] = state->position()[1];
        mj_data_->qpos[2] = state->position()[2];

        data.base_pos[0] = mj_data_->qpos[0];
        data.base_pos[1] = mj_data_->qpos[1];
        data.base_pos[2] = mj_data_->qpos[2];

        // velocity
        mj_data_->qvel[0] = state->velocity()[0];
        mj_data_->qvel[1] = state->velocity()[1];
        mj_data_->qvel[2] = state->velocity()[2];

        data.base_lin_vel[0] = mj_data_->qvel[0];
        data.base_lin_vel[1] = mj_data_->qvel[1];
        data.base_lin_vel[2] = mj_data_->qvel[2];

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