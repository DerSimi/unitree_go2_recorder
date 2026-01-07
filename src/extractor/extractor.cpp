#include "extractor.hpp"

MujocoExtractor::~MujocoExtractor()
{
    running_ = false;
    if (sync_thread_.joinable())
        sync_thread_.join();
}

MujocoExtractor::MujocoExtractor(mjModel *model, mjData *data, ExtractorMode mode, StorageHandler *storage_handler)
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

    sync_thread_ = std::thread([this]()
                               {
        using namespace std::chrono;
        auto next = steady_clock::now();
        while (running_) {
            sync();
            next += milliseconds(2);
            std::this_thread::sleep_until(next);
        } });

    check_sensor();

    spdlog::info("Save your recording by closing the MuJoCo window or pressing Ctrl+C.");
}

// This method will override mj_data and is only called when the simulation mutex is locked.
bool MujocoExtractor::get_rendering_state()
{
    // lock sync_mtx_
    std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);

    if (!has_match_)
        return false;

    // x, y, z
    mj_data_->qpos[0] = last_high_state_match_.base_pos[0];
    mj_data_->qpos[1] = last_high_state_match_.base_pos[1];
    mj_data_->qpos[2] = last_high_state_match_.base_pos[2];

    // Orientation
    mj_data_->qpos[3] = last_low_state_match_.base_quat[0];
    mj_data_->qpos[4] = last_low_state_match_.base_quat[1];
    mj_data_->qpos[5] = last_low_state_match_.base_quat[2];
    mj_data_->qpos[6] = last_low_state_match_.base_quat[3];

    // Base linear and angular velocity
    mj_data_->qvel[0] = last_high_state_match_.base_lin_vel[0];
    mj_data_->qvel[1] = last_high_state_match_.base_lin_vel[1];
    mj_data_->qvel[2] = last_high_state_match_.base_lin_vel[2];

    mj_data_->qvel[3] = last_low_state_match_.base_ang_vel[0];
    mj_data_->qvel[4] = last_low_state_match_.base_ang_vel[1];
    mj_data_->qvel[5] = last_low_state_match_.base_ang_vel[2];

    // Joint position and velocity, and motor commands
    for (int i = 0; i < num_motor_; i++)
    {
        mj_data_->qpos[7 + i] = last_low_state_match_.qpos_joints[i];
        mj_data_->qvel[6 + i] = last_low_state_match_.qvel_joints[i];
        mj_data_->ctrl[i] = last_low_cmd_match_.ctrl[i];
    }

    return true;
}

void MujocoExtractor::sync()
{
    has_match_ = false;

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
    // 1. Get oldes low_state

    // Remove the oldest, we don't want buffer overflows in the lowstate,
    // but we don't care if it happens for the other states, as they are most of the time slower
    // we need to make sure to find a matching partner.
    LowStateData &oldest = low_state_source_.buffer().front();
    last_low_state_match_ = oldest;
    low_state_source_.buffer().pop_front();

    // 2. Get the closest low_cmd in terms of time
    if (!low_cmd_source_.get_closest_match(oldest.stamp, &last_low_cmd_match_))
        return;

    // 3. Get the closest vicon or high state
    if (mode_ == ExtractorMode::VICON)
    { // Vicon
        ViconData vicon_data;
        // This will interpolate position, orientaton
        if (!vicon_source_.get_closest_match(oldest.stamp, &vicon_data))
            return;

        last_high_state_match_.stamp = oldest.stamp;
        last_high_state_match_.base_pos[0] = vicon_data.base_pos[0];
        last_high_state_match_.base_pos[1] = vicon_data.base_pos[1];
        last_high_state_match_.base_pos[2] = vicon_data.base_pos[2];

        last_high_state_match_.base_lin_vel[0] = vicon_data.world_lin_vel[0];
        last_high_state_match_.base_lin_vel[1] = vicon_data.world_lin_vel[1];
        last_high_state_match_.base_lin_vel[2] = vicon_data.world_lin_vel[2];

        // Override low state orientation with VICON data
        last_low_state_match_.base_quat[0] = vicon_data.orientation[3]; // w
        last_low_state_match_.base_quat[1] = vicon_data.orientation[0]; // x
        last_low_state_match_.base_quat[2] = vicon_data.orientation[1]; // y
        last_low_state_match_.base_quat[3] = vicon_data.orientation[2]; // z
    }
    else
    { // SportModeState
        if (!high_state_source_.get_closest_match(oldest.stamp, &last_high_state_match_))
            return;
    }

    storage_handler_->add_state(&last_low_state_match_, &last_low_cmd_match_, &last_high_state_match_,
                                static_cast<double>(oldest.stamp.nanoseconds()) / 1e9);

    has_match_ = true;

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