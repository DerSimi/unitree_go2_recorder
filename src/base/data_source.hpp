#pragma once

#include <chrono>
#include <deque>
#include <mutex>

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"

// Max size of all buffers used for synchronization
#define SYNC_BUFFER_MAX_SIZE 20

// Robot constants
#define NUM_MOTOR 12

#define NOT_IMPLEMENTED throw std::logic_error("Not implemented")

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
  virtual bool get_closest_match(rclcpp::Time &time, void *res);
};

template <typename T>
bool DataSource<T>::get_closest_match(rclcpp::Time &, void *) {
    NOT_IMPLEMENTED;
    return false;
}

inline double interpolate_time(const rclcpp::Time &t_1, const rclcpp::Time &t_2, double y_1, double y_2, const rclcpp::Time &at)
{
  int64_t ns_total = t_2.nanoseconds() - t_1.nanoseconds();
  int64_t dt_at = at.nanoseconds() - t_1.nanoseconds();

  if (ns_total == 0)
    return y_1;

  double ratio = static_cast<double>(dt_at) / static_cast<double>(ns_total);

  return y_1 + (y_2 - y_1) * ratio;
}

inline void interpolate_quat(const rclcpp::Time &t_1, rclcpp::Time &t_2, const Eigen::Quaterniond &quat_1, const Eigen::Quaterniond &quat_2, const rclcpp::Time &at, Eigen::Quaterniond &res)
{
  int64_t ns_total = t_2.nanoseconds() - t_1.nanoseconds();

  if (ns_total == 0)
  {
    res = quat_1;
    return;
  }

  int64_t dt_at = at.nanoseconds() - t_1.nanoseconds();

  double ratio = static_cast<double>(dt_at) / static_cast<double>(ns_total);
  res = quat_1.slerp(ratio, quat_2);
}