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
public:
    StorageHandler(int nq, int nv, int nu);
    ~StorageHandler();

    // add mujoco state to the storage buffer
    void addState(const mjData *data);

    // write data to file
    void storeData(int sig);
};