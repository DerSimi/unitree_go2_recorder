// ViconDataSource: manage background thread lifetime to avoid crashes on client destruction
#include "base/vicon/vicon.hpp"
#include <array>
// Standard
#include <thread>
#include <chrono>
#include <iostream>
#include <unistd.h> // for sleep
#include <rclcpp/rclcpp.hpp>

ViconDataSource::ViconDataSource(const std::string &hostname)
    : hostname_(hostname)
{
}

ViconDataSource::~ViconDataSource()
{
    // Stop streaming thread
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void ViconDataSource::subscribe(rclcpp::Node *node)
{
    running_ = true;
    // Launch background thread for Vicon data loop
    thread_ = std::thread([this]()
                          {
        // Connect to Vicon server
    while (running_ && !client_.IsConnected().Connected)
        {
            auto result = client_.Connect(hostname_).Result;
            if (result != ViconDataStreamSDK::CPP::Result::Success)
            {
                spdlog::warn("Connection to vicon failed, retrying...");
                sleep(1);
            }
        }
        if (!running_) return;
    client_.EnableSegmentData();

        // Streaming loop
    while (running_)
        {
            if (client_.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success)
            {
                spdlog::warn("No vicon frame received, retrying...");
                sleep(1);
                continue;
            }

            std::string subject = "Go2_base";
            std::string root_segment = client_.GetSubjectRootSegmentName(subject).SegmentName;

            auto translation = client_.GetSegmentGlobalTranslation(subject, root_segment);
            auto rotation = client_.GetSegmentGlobalRotationQuaternion(subject, root_segment);

            if (!translation.Occluded && !rotation.Occluded)
            {
                std::array<double, 3> pos_arr;
                std::array<double, 4> quat_arr;
                for (unsigned int i = 0; i < 3; ++i) pos_arr[i] = translation.Translation[i];
                for (unsigned int i = 0; i < 4; ++i) quat_arr[i] = rotation.Rotation[i];
                callback(pos_arr, quat_arr);
            }
            else
            {
                spdlog::warn("Vicon Go2_base not visible.");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    client_.Disconnect(); });
}

void ViconDataSource::callback(const std::array<double, 3> &position, const std::array<double, 4> &orientation)
{
    ViconData data;
    data.timestamp = get_timestamp();

    for (int i = 0; i < 3; i++)
        data.base_pos[i] = position[i] / 1000.0; // Convert mm to m
    for (int i = 0; i < 4; i++)
        data.orientation[i] = orientation[i];

    // Calculate linear velocity
    for (int i = 0; i < 3; i++)
            data.base_lin_vel[i] = 0.0;
    
    // Don't use velocity estimation, there is a issue with timestamps.
    // if (hast_last_position_)
    // {
    //     double dt = data.timestamp - last_timestamp_ / 1000.0; // dt in seconds

    //     for (int i = 0; i < 3; i++)
    //     {
    //         data.base_lin_vel[i] = (position[i] - last_position_[i]) / dt; // m/s
    //     }
    // }
    // else
    // {
    //     for (int i = 0; i < 3; i++)
    //         data.base_lin_vel[i] = 0.0;

    //     hast_last_position_ = true;
    // }

    last_timestamp_ = data.timestamp;
    std::copy(position.begin(), position.end(), last_position_.begin());

    std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);
    buffer_.push_back(data);

    if (buffer_.size() > SYNC_BUFFER_MAX_SIZE)
    {
        buffer_.pop_front();
    }
} // Free mutex
