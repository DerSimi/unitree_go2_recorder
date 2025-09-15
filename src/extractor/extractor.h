#pragma once

#include <iostream>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include <mujoco/mujoco.h>

#include "base/low_state/low_state.hpp"
#include "base/low_cmd/low_cmd.hpp"
#include "base/high_state/high_state.hpp"
#include "base/go2_odometry/go2_odometry.hpp"

#include "storage/storage_handler.h"

using namespace std;

// Topics

#define MOTOR_SENSOR_NUM 3
#define SYNC_BUFFER_MAX_SIZE 100

// Enable to use odometry, the sport state is then ignored.
#define USE_ODOMETRY true

class MujocoExtractor : public rclcpp::Node
{
public:
    MujocoExtractor(mjModel *model, mjData *data, helperData *helper_data);
    ~MujocoExtractor();

    bool GetSynchronizedState(double &out_timestamp);

private:
    // Data sources
    LowStateSource low_state_source_;
    LowCmdSource low_cmd_source_;
    HighStateSource high_state_source_;
    OdometrySource odometry_source_;

    std::mutex buffer_mtx_;

    mjData *mj_data_;
    mjModel *mj_model_;

    helperData *helper_data_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;
    int have_imu_ = false;
    int have_frame_sensor_ = false;

    void insertSynchronizedData(const LowCmdData &cmd, const LowStateData &low_state, const HighStateData &high_state);
    void checkSensor();
};
