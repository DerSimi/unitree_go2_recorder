#pragma once

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <pthread.h>
#include <filesystem>

#include <spdlog/spdlog.h>

#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"

#include "common.hpp"
#include "extractor/extractor.hpp"
#include "storage/storage_handler.hpp"

extern "C"
{
#include <sys/errno.h>
#include <unistd.h>
}

namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

#define K_ERROR_LENGTH 1024

class Simulator
{
public:
    Simulator(ExtractorMode mode, const std::string &storage_path, std::string model_path, const std::string &vicon_ip, const std::string &vicon_subject);
    ~Simulator() = default;
private:
    // General extractor functionality
    std::shared_ptr<MujocoExtractor> extractor_node_;
    ExtractorMode mode_;

    std::string storage_path_;

    // For better vicon handling only
    std::string vicon_ip_;
    std::string vicon_subject_;

    std::unique_ptr<StorageHandler> storage_handler_;

    // Simulator specific
    mjModel *m_;
    mjData *d_ = nullptr;
    std::unique_ptr<mj::Simulate> sim_;

    //static instance for signal handler
    static Simulator* instance_;
    static void sig_handler(int signum);

    mjModel *load_model(const char *file);
    void physics_loop();
    void physics_thread(const char *file);
    void extractor_thread();

    // Trigger storage
    void terminate();
};