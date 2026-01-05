#pragma once

#include <iostream>
#include <vector>

#include <cnpy.h>
#include <mujoco/mujoco.h>

#include <spdlog/spdlog.h>

#include "common.hpp"

#include "base/low_state/low_state.hpp"
#include "base/low_cmd/low_cmd.hpp"
#include "base/high_state/high_state.hpp"

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
    void add_state(const LowStateData* low_state_data, const LowCmdData* low_cmd_data, const HighStateData* high_state_data, double timestamp);

    // write data to file
    void store_data(const char *filename);
};