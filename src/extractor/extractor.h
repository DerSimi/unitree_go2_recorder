#pragma once

#include <iostream>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <mujoco/mujoco.h>

#include "storage/storage_handler.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

// Topics
#define TOPIC_LOWSTATE "/lowstate"
#define TOPIC_HIGHSTATE "/sportmodestate"
#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_ODOMETRY "/tf"
#define TOPIC_ODOMETRY_FILTERED "/odometry/filtered"

#define MOTOR_SENSOR_NUM 3
#define SYNC_BUFFER_MAX_SIZE 100

// Enable to use odometry, the sport state is then ignored.
#define USE_ODOMETRY true

struct LowStateData
{
    double timestamp;
    std::vector<mjtNum> qpos_joints; // 12 joint angles
    std::vector<mjtNum> qvel_joints; // 12 joint velocities
    std::vector<float> tau_est;      // 12 estimated motor torques
    std::vector<float> q_raw;        // 12 raw joint positions
    std::vector<float> dq_raw;       // 12 raw joint velocities
    mjtNum base_quat[4];             // base rotation
    mjtNum base_ang_vel[3];          // base angular velocity

    LowStateData(size_t num_motor)
        : qpos_joints(num_motor),
          qvel_joints(num_motor),
          tau_est(num_motor),
          q_raw(num_motor),
          dq_raw(num_motor)
    {
    }
};

struct HighStateData
{
    double timestamp;
    mjtNum base_pos[3];     // base position
    mjtNum base_lin_vel[3]; // base linear velocity
};

struct OdometryData
{
    double timestamp;
    mjtNum base_pos[3]; // base position
};

struct OdometryFilteredData
{
    double timestamp;
    mjtNum base_lin_vel[3]; // base linear velocity
};

struct LowCmdData
{
    double timestamp;
    std::vector<mjtNum> ctrl;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
    std::vector<float> kp;
    std::vector<float> kd;

    LowCmdData(size_t num_motor)
        : ctrl(num_motor),
          q(num_motor),
          dq(num_motor),
          tau(num_motor),
          kp(num_motor),
          kd(num_motor)
    {
    }
};

class MujocoExtractor : public rclcpp::Node
{
public:
    MujocoExtractor(mjModel *model, mjData *data, helperData *helper_data);
    ~MujocoExtractor();

    bool GetSynchronizedState(double &out_timestamp, bool disabled = false);

private:
    rclcpp::CallbackGroup::SharedPtr low_state_group_;
    rclcpp::CallbackGroup::SharedPtr low_cmd_group_;
    rclcpp::CallbackGroup::SharedPtr high_state_group_;
    rclcpp::CallbackGroup::SharedPtr odometry_group_;
    rclcpp::CallbackGroup::SharedPtr odometry_filtered_group_;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr high_state_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_filtered_sub_;

    std::deque<LowStateData> low_state_buffer_;
    std::deque<HighStateData> high_state_buffer_;
    std::deque<LowCmdData> low_cmd_buffer_;
    std::deque<OdometryData> odometry_buffer_;
    std::deque<OdometryFilteredData> odometry_filtered_buffer_;

    std::mutex mtx_;

    mjData *mj_data_;
    mjModel *mj_model_;

    helperData *helper_data_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;

    // Handler
    void lowCmdCallback(const unitree_go::msg::LowCmd::SharedPtr msg);
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void highStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    void odometryCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void odometryFilteredCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void insertSynchronizedData(const LowCmdData &cmd, const LowStateData &low_state, const HighStateData &high_state);

    void checkSensor();
};
