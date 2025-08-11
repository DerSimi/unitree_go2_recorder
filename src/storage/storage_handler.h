#pragma once

#include <iostream>
#include <vector>

#include <cnpy.h>
#include <mujoco/mujoco.h>

using namespace std;

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
    void addState(const mjData *data, double timestamp = 0.0);

    // write data to file
    void storeData();
};