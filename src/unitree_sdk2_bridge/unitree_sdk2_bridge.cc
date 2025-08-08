#include "unitree_sdk2_bridge.h"

UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();
    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowStateHandler, this, placeholders::_1), 1);

    // lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowStateGo, this);

    // high_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    // high_state_puber_->InitChannel();

    // HighStatePuberThreadPtr = CreateRecurrentThreadEx("highstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishHighState, this);
}

UnitreeSdk2Bridge::~UnitreeSdk2Bridge()
{
}

void UnitreeSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            // mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
            //                     cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
            //                     cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

// nu ctrl dimension
// nq qpos dimension
// nv qvel dimension

void UnitreeSdk2Bridge::LowStateHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowState_ *state = (const unitree_go::msg::dds_::LowState_ *)msg;

    if (mj_data_)
    {
        // cout << "low state called" << endl;
        // cout << mj_data_->qpos[2] << endl;

        // cout << "x: " << mj_data_->qpos[0] << ", "
        //      << "y: " << mj_data_->qpos[1] << ", "
        //      << "z: " << mj_data_->qpos[2] << endl;

        // cout << "quat: ["
        // << mj_data_->qpos[3] << ", "
        // << mj_data_->qpos[4] << ", "
        // << mj_data_->qpos[5] << ", "
        // << mj_data_->qpos[6] << "]" << endl;

        // fix position
        mj_data_->qpos[0] = -0.05;
        mj_data_->qpos[1] = 0;
        mj_data_->qpos[2] = 0.4;

        // fix rotation
        // Write quaternion values into mj_data_->qpos
        mj_data_->qpos[3] = 0.998648;
        mj_data_->qpos[4] = 7.44033e-07;
        mj_data_->qpos[5] = -0.0519753;
        mj_data_->qpos[6] = -1.9382e-05;

        // for (int i = 0; i < num_motor_; i++)
        // {
        //     std::cout << "Motor " << i << ": "
        //               << "q = " << state->motor_state()[i].q() << ", "
        //               << "dq = " << state->motor_state()[i].dq() << ", "
        //               << "tau_est = " << state->motor_state()[i].tau_est() << endl;
        // }

        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->qpos[7 + i] = state->motor_state()[i].q();
        }
    }
}

// void UnitreeSdk2Bridge::PublishLowStateGo()
// {
//     if (mj_data_)
//     {
//         for (int i = 0; i < num_motor_; i++)
//         {
//             low_state_go_.motor_state()[i].q() = mj_data_->sensordata[i];
//             low_state_go_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
//             low_state_go_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
//         }

//         if (have_frame_sensor_)
//         {
//             low_state_go_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
//             low_state_go_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
//             low_state_go_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
//             low_state_go_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

// 	    double w = low_state_go_.imu_state().quaternion()[0];
// 	    double x = low_state_go_.imu_state().quaternion()[1];
// 	    double y = low_state_go_.imu_state().quaternion()[2];
// 	    double z = low_state_go_.imu_state().quaternion()[3];

// 	    low_state_go_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
// 	    low_state_go_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
// 	    low_state_go_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

//             low_state_go_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
//             low_state_go_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
//             low_state_go_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

//             low_state_go_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
//             low_state_go_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
//             low_state_go_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
//         }

//         low_state_go_puber_->Write(low_state_go_);
//     }
// }

// void UnitreeSdk2Bridge::PublishHighState()
// {
//     if (mj_data_ && have_frame_sensor_)
//     {

//         high_state_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
//         high_state_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
//         high_state_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];

//         high_state_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
//         high_state_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
//         high_state_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];

//         high_state_puber_->Write(high_state_);
//     }
// }

void UnitreeSdk2Bridge::Run()
{
    while (1)
    {
        sleep(2);
    }
}

// void UnitreeSdk2Bridge::PrintSceneInformation()
// {
//     cout << endl;

//     cout << "<<------------- Link ------------->> " << endl;
//     for (int i = 0; i < mj_model_->nbody; i++)
//     {
//         const char *name = mj_id2name(mj_model_, mjOBJ_BODY, i);
//         if (name)
//         {
//             cout << "link_index: " << i << ", "
//                  << "name: " << name
//                  << endl;
//         }
//     }
//     cout << endl;

//     cout << "<<------------- Joint ------------->> " << endl;
//     for (int i = 0; i < mj_model_->njnt; i++)
//     {
//         const char *name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
//         if (name)
//         {
//             cout << "joint_index: " << i << ", "
//                  << "name: " << name
//                  << endl;
//         }
//     }
//     cout << endl;

//     cout << "<<------------- Actuator ------------->> " << endl;
//     for (int i = 0; i < mj_model_->nu; i++)
//     {
//         const char *name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
//         if (name)
//         {
//             cout << "actuator_index: " << i << ", "
//                  << "name: " << name
//                  << endl;
//         }
//     }
//     cout << endl;

//     cout << "<<------------- Sensor ------------->> " << endl;
//     int index = 0;
//     // 多维传感器，输出第一维的index
//     for (int i = 0; i < mj_model_->nsensor; i++)
//     {
//         const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
//         if (name)
//         {
//             cout << "sensor_index: " << index << ", "
//                  << "name: " << name << ", "
//                  << "dim: " << mj_model_->sensor_dim[i]
//                  << endl;
//         }
//         index = index + mj_model_->sensor_dim[i];
//     }
//     cout << endl;
// }

// nu ctrl dimension
// nq qpos dimension
// nv qvel dimension

/*
12 Motoren, also ctrl = 12

qpos dim ist 19:
7 für basis: position, quat
wir haben 12 gelenke, pro gelenk ein winkel, 7 + 12 = 19

nv qvel dimension ist 18:
6 für basis: linear velocity, angular velocity
wir haben 12 gelenke, pro gelenk eine Geschwindigkeit, 6 + 12 = 18

*/

void UnitreeSdk2Bridge::CheckSensor()
{
    num_motor_ = mj_model_->nu;

    std::cout << "num_motor_: " << num_motor_ << std::endl;

    std::cout << "mj_model_->qpos dim: " << mj_model_->nq << std::endl;
    std::cout << "mj_model_->qvel dim: " << mj_model_->nv << std::endl;

    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0)
        {
            have_imu_ = true;
        }
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }
}