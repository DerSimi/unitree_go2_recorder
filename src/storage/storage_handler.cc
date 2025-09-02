#include "storage_handler.h"

StorageHandler::StorageHandler(int nq, int nv, int nu) : nq_(nq), nv_(nv), nu_(nu)
{
    // Reserve capacity for 10.000 states in the storage buffer
    // 8 for all helper fields
    data_.reserve((nq_ + nv_ + nu_ + 8 * nu_) * 10000);
    time_.reserve(10000);
}

StorageHandler::~StorageHandler()
{
}

void StorageHandler::addState(const mjData *mj_data, const helperData *helper_data, double timestamp)
{
    data_.insert(data_.end(), mj_data->qpos, mj_data->qpos + nq_);
    data_.insert(data_.end(), mj_data->qvel, mj_data->qvel + nv_);
    data_.insert(data_.end(), mj_data->ctrl, mj_data->ctrl + nu_);

    //Insert helper fields
    data_.insert(data_.end(), helper_data->q.begin(), helper_data->q.end());
    data_.insert(data_.end(), helper_data->dq.begin(), helper_data->dq.end());
    data_.insert(data_.end(), helper_data->tau.begin(), helper_data->tau.end());
    data_.insert(data_.end(), helper_data->kp.begin(), helper_data->kp.end());
    data_.insert(data_.end(), helper_data->kd.begin(), helper_data->kd.end());
    
    data_.insert(data_.end(), helper_data->tau_est.begin(), helper_data->tau_est.end());
    data_.insert(data_.end(), helper_data->q_raw.begin(), helper_data->q_raw.end());
    data_.insert(data_.end(), helper_data->dq_raw.begin(), helper_data->dq_raw.end());

    time_.push_back(timestamp);
}

void StorageHandler::storeData()
{
    cout << "Writing " << data_.size() / (nq_ + nv_ + nu_ + 8 * nu_) << " states to file..." << endl;
    // Write data to file
    cnpy::npz_save("storage.npy", "state", data_);

    cnpy::npz_save("storage.npy", "timestamp", time_, "a");

    std::vector<double> dt;
    if (time_.size() > 1) {
        dt.reserve(time_.size() - 1);
        for (size_t i = 1; i < time_.size(); ++i) {
            dt.push_back(time_[i] - time_[i - 1]);
        }
    }

    cnpy::npz_save("storage.npy", "dt", dt, "a");
}