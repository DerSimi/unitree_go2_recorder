#pragma once

#include <iostream>
#include <deque>
#include <chrono>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <mujoco/mujoco.h>

#include <spdlog/spdlog.h>

#include "base/low_state/low_state.hpp"
#include "base/low_cmd/low_cmd.hpp"
#include "base/high_state/high_state.hpp"
#include "base/vicon/vicon.hpp"

#include "storage/storage_handler.hpp"

using namespace std;

#define MOTOR_SENSOR_NUM 3

enum class ExtractorMode {
    HIGHSTATE,
    VICON
};

class MujocoExtractor : public rclcpp::Node
{
public:
    MujocoExtractor(mjModel *model, mjData *data, ExtractorMode mode, StorageHandler* storage_handler);
    ~MujocoExtractor();

    // This function is only triggered for visualization.
    bool get_rendering_state();
    // ... it relies on the data being stored by store_data, which is triggered independently
    // to avoid interference by simulation.
    void sync();

private:
    // Data sources
    LowStateSource low_state_source_;
    LowCmdSource low_cmd_source_;
    HighStateSource high_state_source_;
    ViconDataSource vicon_source_;

    // Matches
    LowStateData last_low_state_match_{NUM_MOTOR};
    LowCmdData last_low_cmd_match_{NUM_MOTOR};
    HighStateData last_high_state_match_;
    bool has_match_ = false;

    mjData *mj_data_;
    mjModel *mj_model_;

    ExtractorMode mode_;

    StorageHandler *storage_handler_;

    std::thread sync_thread_;
    std::atomic<bool> running_{true};

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;
    int have_imu_ = false;
    int have_frame_sensor_ = false;

    void insertSynchronizedData(const LowCmdData &cmd, const LowStateData &low_state, const HighStateData &high_state);
    void check_sensor();
};
