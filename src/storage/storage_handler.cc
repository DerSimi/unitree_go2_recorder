#include "storage_handler.h"

StorageHandler::StorageHandler(int nq, int nv, int nu) : nq_(nq), nv_(nv), nu_(nu)
{
    // Reserve capacity for 10.000 states in the storage buffer
    data_.reserve((nq_ + nv_ + nu_) * 10000);
    time_.reserve(10000);
}

StorageHandler::~StorageHandler()
{
}

void StorageHandler::addState(const mjData *mj_data, double timestamp)
{
    data_.insert(data_.end(), mj_data->qpos, mj_data->qpos + nq_);
    data_.insert(data_.end(), mj_data->qvel, mj_data->qvel + nv_);
    data_.insert(data_.end(), mj_data->ctrl, mj_data->ctrl + nu_);
    time_.push_back(timestamp);
}

void StorageHandler::storeData()
{
    cout << "Writing " << data_.size() / (nq_ + nv_ + nu_) << " states to file..." << endl;
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