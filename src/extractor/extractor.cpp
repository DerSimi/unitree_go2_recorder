#include "extractor.hpp"

MujocoExtractor::~MujocoExtractor()
{
    running_ = false;
    if (sync_thread_.joinable())
        sync_thread_.join();
}

MujocoExtractor::MujocoExtractor(mjModel *model, mjData *data, ExtractorMode mode, StorageHandler* storage_handler)
    : rclcpp::Node("mujoco_extractor_node"),
      mj_model_(model),
      mj_data_(data),
      mode_(mode),
      storage_handler_(storage_handler),
      low_state_source_(),
      low_cmd_source_(data),
      high_state_source_(),
      vicon_source_("10.0.0.20")
{
    // New state representation
    low_state_source_.subscribe(this);
    low_cmd_source_.subscribe(this);

    switch (mode_)
    {
    case ExtractorMode::HIGHSTATE:
        spdlog::info("Using SportStateMode for base estimation, note this is not available in low state mode.");
        high_state_source_.subscribe(this);
        break;
    case ExtractorMode::VICON:
        spdlog::info("Using VICON for base estimation.");
        // This is not a ros topic, but we still stick to this interface
        vicon_source_.subscribe(this);
        break;
    default:
        break;
    }

    sync_thread_ = std::thread([this]() {
        using namespace std::chrono;
        auto next = steady_clock::now();
        while (running_) {
            sync();
            next += milliseconds(2);
            std::this_thread::sleep_until(next);
        }
    });

    check_sensor();

    spdlog::info("Save your recording by closing the MuJoCo window or pressing Ctrl+C.");
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
    // for (int i = 0; i < num_motor_; i++)
    // {
    //     // Low command
    //     helper_data_->q[i] = cmd.q[i];
    //     helper_data_->dq[i] = cmd.dq[i];
    //     helper_data_->tau[i] = cmd.tau[i];
    //     helper_data_->kp[i] = cmd.kp[i];
    //     helper_data_->kd[i] = cmd.kd[i];

    //     // Low state
    //     helper_data_->tau_est[i] = low_state.tau_est[i];
    //     helper_data_->q_raw[i] = low_state.q_raw[i];
    //     helper_data_->dq_raw[i] = low_state.dq_raw[i];
    // }
}

bool MujocoExtractor::get_rendering_state()
{
    return false;
    // if (!mj_data_ && have_frame_sensor_)
    //     return false;

    // std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);

    // if (mode_ == ExtractorMode::HIGHSTATE && high_state_source_.buffer().empty())
    // {
    //     return false;
    // }

    // if (mode_ == ExtractorMode::VICON && vicon_source_.buffer().empty())
    // {
    //     return false;
    // }

    // if (low_cmd_source_.buffer().empty() || low_state_source_.buffer().empty())
    // {
    //     return false;
    // }

    // if you now go for the oldest lowststate,

    // const auto &ref_low_cmd = low_cmd_source_.buffer().front();
    // double T_ref = ref_low_cmd.timestamp;

    // // Get the low state first
    // auto it_low = low_state_source_.buffer().rbegin();
    // while (it_low != low_state_source_.buffer().rend() && it_low->timestamp > T_ref)
    // {
    //     ++it_low;
    // }

    // if (it_low == low_state_source_.buffer().rend())
    // {
    //     low_cmd_source_.buffer().pop_front();
    //     return false;
    // }

    // LowStateData *match_low_state = &(*it_low);

    // // Now get the high state
    // HighStateData high_state_data;

    // if (mode_ == ExtractorMode::HIGHSTATE)
    // {
    //     auto it_high = high_state_source_.buffer().rbegin();
    //     while (it_high != high_state_source_.buffer().rend() && it_high->timestamp > T_ref)
    //     {
    //         ++it_high;
    //     }

    //     if (it_high == high_state_source_.buffer().rend())
    //     {
    //         low_cmd_source_.buffer().pop_front();
    //         return false;
    //     }

    //     high_state_data = *it_high;

    //     high_state_source_.buffer().erase(high_state_source_.buffer().begin(), it_high.base() - 1);
    // }
    // else if (mode_ == ExtractorMode::VICON) // Use VICON data
    // {
    //     auto it_vicon = vicon_source_.buffer().rbegin();
    //     while (it_vicon != vicon_source_.buffer().rend() && it_vicon->timestamp > T_ref)
    //     {
    //         ++it_vicon;
    //     }

    //     if (it_vicon == vicon_source_.buffer().rend())
    //     {
    //         low_cmd_source_.buffer().pop_front();
    //         return false;
    //     }

    //     ViconData vicon_data = *it_vicon;

    //     high_state_data.timestamp = T_ref;

    //     // Store the result
    //     // Base pos
    //     high_state_data.base_pos[0] = vicon_data.base_pos[0];
    //     high_state_data.base_pos[1] = vicon_data.base_pos[1];
    //     high_state_data.base_pos[2] = vicon_data.base_pos[2];

    //     high_state_data.base_lin_vel[0] = vicon_data.base_lin_vel[0];
    //     high_state_data.base_lin_vel[1] = vicon_data.base_lin_vel[1];
    //     high_state_data.base_lin_vel[2] = vicon_data.base_lin_vel[2];

    //     // Override low state orientation with VICON data
    //     match_low_state->base_quat[0] = vicon_data.orientation[3]; // w
    //     match_low_state->base_quat[1] = vicon_data.orientation[0]; // x
    //     match_low_state->base_quat[2] = vicon_data.orientation[1]; // y
    //     match_low_state->base_quat[3] = vicon_data.orientation[2]; // z

    //     vicon_source_.buffer().erase(vicon_source_.buffer().begin(), it_vicon.base() - 1);
    // }

    // // Insert data into mj_data
    // insertSynchronizedData(ref_low_cmd, *match_low_state, high_state_data);

    // out_timestamp = T_ref;
    // low_cmd_source_.buffer().pop_front();

    // low_state_source_.buffer().erase(low_state_source_.buffer().begin(), it_low.base() - 1);

    return false;
}

void MujocoExtractor::sync()
{
    if (!mj_data_ && have_frame_sensor_)
        return;

    // lock sync_mtx_
    std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);

    if (mode_ == ExtractorMode::HIGHSTATE && high_state_source_.buffer().empty())
    {
        return;
    }

    if (mode_ == ExtractorMode::VICON && vicon_source_.buffer().empty())
    {
        return;
    }

    if (low_cmd_source_.buffer().empty() || low_state_source_.buffer().empty())
    {
        return;
    }
    //1. Get oldes low_state

    // Remove the oldest, we don't want buffer overflows in the lowstate, 
    // but we don't care if it happens for the other states, as they are most of the time slower
    // we need to make sure to find a matching partner.
    LowStateData& oldest = low_state_source_.buffer().front();
    low_state_source_.buffer().pop_front();
    
    //2. Get the closest low_cmd in terms of time,
    LowCmdData cmd_data(NUM_MOTOR);
    low_cmd_source_.get_closest_match(oldest.stamp, &cmd_data);

    HighStateData high_state_data;

    //3. Get the closest vicon or high state
    if(mode_ == ExtractorMode::VICON) 
    { // Vicon
        ViconData vicon_data;
        // This will interpolate position, orientaton
        vicon_source_.get_closest_match(oldest.stamp, &vicon_data);

        high_state_data.stamp = oldest.stamp;
        high_state_data.base_pos[0] = vicon_data.base_pos[0];
        high_state_data.base_pos[1] = vicon_data.base_pos[1];
        high_state_data.base_pos[2] = vicon_data.base_pos[2];

        high_state_data.base_lin_vel[0] = vicon_data.world_lin_vel[0];
        high_state_data.base_lin_vel[1] = vicon_data.world_lin_vel[1];
        high_state_data.base_lin_vel[2] = vicon_data.world_lin_vel[2];
    } else { // SportModeState
        high_state_source_.get_closest_match(oldest.stamp, &high_state_data);
    }


} // free sync_mtx_

void MujocoExtractor::check_sensor()
{
    num_motor_ = mj_model_->nu;
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