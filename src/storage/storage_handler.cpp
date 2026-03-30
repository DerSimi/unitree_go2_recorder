#include "storage_handler.hpp"

StorageHandler::StorageHandler(int nq, int nv, int nu) : nq_(nq), nv_(nv), nu_(nu), idx_(0)
{
    // Reserve capacity for 10.000 states in the storage buffer
    // 8 for all helper fields
    data_.reserve((nq_ + nv_ + nu_ + 8 * nu_) * 10000);
    time_.reserve(10000);
}

StorageHandler::~StorageHandler()
{
}

void StorageHandler::add_state(const LowStateData *low_state_data, const LowCmdData *low_cmd_data, const HighStateData *high_state_data, double timestamp)
{
    // qpos: Base pos (3), base ori (4), joint pos (12): 19
    std::vector<mjtNum> qpos;
    qpos.reserve(nq_);

    qpos.push_back(high_state_data->base_pos[0]);
    qpos.push_back(high_state_data->base_pos[1]);
    qpos.push_back(high_state_data->base_pos[2]);

    qpos.push_back(low_state_data->base_quat[0]);
    qpos.push_back(low_state_data->base_quat[1]);
    qpos.push_back(low_state_data->base_quat[2]);
    qpos.push_back(low_state_data->base_quat[3]);

    qpos.insert(qpos.end(), low_state_data->qpos_joints.begin(), low_state_data->qpos_joints.end());
    data_.insert(data_.end(), qpos.begin(), qpos.end());

    // qvel: Base lin vel (3), base ang vel (3), joint vel (12): 18
    std::vector<mjtNum> qvel;
    qvel.reserve(nv_);

    qvel.push_back(high_state_data->base_lin_vel[0]);
    qvel.push_back(high_state_data->base_lin_vel[1]);
    qvel.push_back(high_state_data->base_lin_vel[2]);

    qvel.push_back(low_state_data->base_ang_vel[0]);
    qvel.push_back(low_state_data->base_ang_vel[1]);
    qvel.push_back(low_state_data->base_ang_vel[2]);

    qvel.insert(qvel.end(), low_state_data->qvel_joints.begin(), low_state_data->qvel_joints.end());
    data_.insert(data_.end(), qvel.begin(), qvel.end());

    // ctrl: motor commands (12): 12
    data_.insert(data_.end(), low_cmd_data->ctrl.begin(), low_cmd_data->ctrl.end());

    // LowCmd additional data
    data_.insert(data_.end(), low_cmd_data->q.begin(), low_cmd_data->q.end());
    data_.insert(data_.end(), low_cmd_data->dq.begin(), low_cmd_data->dq.end());
    data_.insert(data_.end(), low_cmd_data->tau.begin(), low_cmd_data->tau.end());
    data_.insert(data_.end(), low_cmd_data->kp.begin(), low_cmd_data->kp.end());
    data_.insert(data_.end(), low_cmd_data->kd.begin(), low_cmd_data->kd.end());

    // LowState additional data
    data_.insert(data_.end(), low_state_data->tau_est.begin(), low_state_data->tau_est.end());
    data_.insert(data_.end(), low_state_data->q_raw.begin(), low_state_data->q_raw.end());
    data_.insert(data_.end(), low_state_data->dq_raw.begin(), low_state_data->dq_raw.end());

    // LowState foot force
    std::vector<mjtNum> foot_force;
    foot_force.reserve(4);

    foot_force.push_back(low_state_data->foot_force[0]);
    foot_force.push_back(low_state_data->foot_force[1]);
    foot_force.push_back(low_state_data->foot_force[2]);
    foot_force.push_back(low_state_data->foot_force[3]);

    data_.insert(data_.end(), foot_force.begin(), foot_force.end());

    time_.push_back(timestamp);

    // Increase state idx for marker
    idx_++;
}

void StorageHandler::add_marker()
{
    int current_idx = idx_.load();
    markers_.push_back(current_idx);

    spdlog::info("Added marker at index {}.", current_idx);
}

void StorageHandler::store_data(const char *filename)
{
    if (data_.size() == 0)
    {
        spdlog::warn("No data was recorded!");
        return;
    }

    spdlog::info("Exporting {} data points to {}...",
                 data_.size() / (nq_ + nv_ + nu_ + 8 * nu_), filename);
    // Write data to file
    cnpy::npz_save(filename, "state", data_);

    cnpy::npz_save(filename, "timestamp", time_, "a");

    std::vector<double> dt;
    if (time_.size() > 1)
    {
        dt.reserve(time_.size() - 1);
        for (size_t i = 1; i < time_.size(); ++i)
        {
            dt.push_back(time_[i] - time_[i - 1]);
        }
    }

    cnpy::npz_save(filename, "dt", dt, "a");

    if (!markers_.empty())
    {
        std::ostringstream oss;
        oss << "Markers at indices: ";
        for (size_t i = 0; i < markers_.size(); ++i)
        {
            oss << markers_[i];
            if (i != markers_.size() - 1)
                oss << ", ";
        }
        spdlog::info(oss.str());

        cnpy::npz_save(filename, "markers", markers_, "a");
    }

    spdlog::info("Recording stored as {}.", filename);
}