#pragma once

#include <iostream>
#include <vector>

#include <cnpy.h>
#include <mujoco/mujoco.h>

#include <spdlog/spdlog.h>

#include "common.hpp"

struct helperData
{
    std::vector<float> tau_est;
    std::vector<float> q_raw;
    std::vector<float> dq_raw;

    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
    std::vector<float> kp;
    std::vector<float> kd;

    helperData(size_t size = 0)
    {
        tau_est.resize(size);
        q_raw.resize(size);
        dq_raw.resize(size);
        q.resize(size);
        dq.resize(size);
        tau.resize(size);
        kp.resize(size);
        kd.resize(size);
    }
};

class StorageHandler
{
private:
    int nq_;
    int nv_;
    int nu_;
    std::vector<mjtNum> data_;
    std::vector<double> time_;

public:
    StorageHandler(int nq, int nv, int nu);
    ~StorageHandler();

    // add mujoco state to the storage buffer
    void add_state(const mjData *data, const helperData *helper_data, double timestamp = 0.0);

    // write data to file
    void store_data(const char *filename);
};