#pragma once

#include <chrono>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

// Max size of all buffers used for synchronization
#define SYNC_BUFFER_MAX_SIZE 100

// Robot constants
#define NUM_MOTOR 12

// Base synchronization mutex for all data sources
class DataSourceBase
{
public:
  static std::mutex sync_mtx_;
};

template <typename T>
class DataSource : public DataSourceBase
{
public:
  virtual ~DataSource() = default;
  virtual void subscribe(rclcpp::Node *node) = 0;
  virtual std::deque<T> &buffer() = 0;
};

// Small helper functions
inline double get_timestamp()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}