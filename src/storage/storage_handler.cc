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

void StorageHandler::addState(const mjData *mj_data, double time)
{
    data_.insert(data_.end(), mj_data->qpos, mj_data->qpos + nq_);
    data_.insert(data_.end(), mj_data->qvel, mj_data->qvel + nv_);
    data_.insert(data_.end(), mj_data->ctrl, mj_data->ctrl + nu_);
    time_.push_back(mj_data->time);
}

void StorageHandler::storeData()
{
    cout << "Writing data to file..." << endl;
    // Write data to file
    cnpy::npz_save("storage.npy", "state", data_);
}