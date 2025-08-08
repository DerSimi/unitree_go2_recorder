#include "extractor.h"

MujocoExtractor::~MujocoExtractor()
{
}

MujocoExtractor::MujocoExtractor(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();
    low_cmd_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_suber_->InitChannel(bind(&MujocoExtractor::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_suber_->InitChannel(bind(&MujocoExtractor::LowStateHandler, this, placeholders::_1), 1);

    high_state_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_state_suber_->InitChannel(bind(&MujocoExtractor::HighStateHandler, this, placeholders::_1), 1);
}

void MujocoExtractor::LowCmdGoHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
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
    const unitree_go::msg::dds_::LowState_ *state = (const unitree_go::msg::dds_::LowState_ *)msg;

    if (mj_data_)
    {
        // Inject IMU state
        if (have_frame_sensor_)
        {
            // Rotation quaternion
            mj_data_->qpos[3] = state->imu_state().quaternion()[0];
            mj_data_->qpos[4] = state->imu_state().quaternion()[1];
            mj_data_->qpos[5] = state->imu_state().quaternion()[2];
            mj_data_->qpos[6] = state->imu_state().quaternion()[3];
            // Angular velocity
            mj_data_->qvel[0] = state->imu_state().gyroscope()[0];
            mj_data_->qvel[1] = state->imu_state().gyroscope()[1];
            mj_data_->qvel[2] = state->imu_state().gyroscope()[2];

            // Linear velocity is not in here, only acceleration!!!
        }

        // Inject motor state
        for (int i = 0; i < num_motor_; i++)
        {
            // angle
            mj_data_->qpos[7 + i] = state->motor_state()[i].q();
            // angular velocity
            mj_data_->qvel[7 + i] = state->motor_state()[i].dq();
        }
    }
}

void MujocoExtractor::HighStateHandler(const void *msg)
{
    const unitree_go::msg::dds_::SportModeState_ *state = (const unitree_go::msg::dds_::SportModeState_ *)msg;

    //Inject position and velocity into mujoco data
    if(mj_data_ && have_frame_sensor_) {
        // position
        mj_data_->qpos[0] = state->position()[0];
        mj_data_->qpos[1] = state->position()[1];
        mj_data_->qpos[2] = state->position()[2];

        // velocity
        mj_data_->qvel[0] = state->velocity()[0];
        mj_data_->qvel[1] = state->velocity()[1];
        mj_data_->qvel[2] = state->velocity()[2];
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

    std::cout << "num_motor_: " << num_motor_ << std::endl;

    std::cout << "mj_model_->qpos dim: " << mj_model_->nq << std::endl;
    std::cout << "mj_model_->qvel dim: " << mj_model_->nv << std::endl;

    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }
}