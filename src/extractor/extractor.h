#pragma once

#include <iostream>
#include <deque>
#include <chrono>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <mujoco/mujoco.h>

#include "storage/storage_handler.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3
#define SYNC_BUFFER_MAX_SIZE 100

struct LowStateData
{
    double timestamp;
    std::vector<mjtNum> qpos_joints; // 12 joint angles
    std::vector<mjtNum> qvel_joints; // 12 joint velocities
    std::vector<float> tau_est;    // 12 estimated motor torques
    std::vector<float> q_raw;      // 12 raw joint positions
    std::vector<float> dq_raw;     // 12 raw joint velocities
    mjtNum base_quat[4];             // base rotation
    mjtNum base_ang_vel[3];          // base angular velocity
};

struct HighStateData
{
    double timestamp;
    mjtNum base_pos[3];     // base position
    mjtNum base_lin_vel[3]; // base linear velocity
};

struct LowCmdData
{
    double timestamp;
    std::vector<mjtNum> ctrl; // 12 control commands
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
    std::vector<float> kp;
    std::vector<float> kd;
};

class MujocoExtractor
{
public:
    MujocoExtractor(mjModel *model, mjData *data, helperData* helper_data);
    ~MujocoExtractor();

    void LowCmdHandler(const void *msg);
    void LowStateHandler(const void *msg);
    void HighStateHandler(const void *msg);
    bool GetSynchronizedState(double &out_timestamp);

    void Run();

private:
    void CheckSensor();

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_suber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_suber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_suber_;

    std::deque<LowStateData> low_state_buffer_;
    std::deque<HighStateData> high_state_buffer_;
    std::deque<LowCmdData> low_cmd_buffer_;

    std::mutex mtx_;

    mjData *mj_data_;
    mjModel *mj_model_;

    helperData *helper_data_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
};
