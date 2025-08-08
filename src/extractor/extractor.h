#pragma once

#include <iostream>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <mujoco/mujoco.h>

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3

class MujocoExtractor
{
public:
    MujocoExtractor(mjModel *model, mjData *data);
    ~MujocoExtractor();

    void LowCmdGoHandler(const void *msg);
    void LowStateHandler(const void *msg);
    void HighStateHandler(const void *msg);

    void Run();

private:
    void CheckSensor();

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_suber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_suber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_suber_;

    mjData *mj_data_;
    mjModel *mj_model_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
};
