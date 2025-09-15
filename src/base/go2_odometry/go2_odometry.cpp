#include "base/go2_odometry/go2_odometry.hpp"

void OdometrySource::subscribe(rclcpp::Node *node)
{
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    topic_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        TOPIC_ODOMETRY_FILTERED, rclcpp::SensorDataQoS(),
        std::bind(&OdometrySource::callback, this, std::placeholders::_1), sub_opt);
}

void OdometrySource::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!initial_rotation_captured_)
    {
        // We need to set odom to the mujoco world frame: Start position is zero, and rotation is pointing to the x-axis.
        Eigen::Vector3d initial_odom_pos(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        Eigen::Quaterniond initial_odom_quat(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        Eigen::Isometry3d initial_odom_transform = Eigen::Isometry3d::Identity();
        initial_odom_transform.translate(initial_odom_pos);
        initial_odom_transform.rotate(initial_odom_quat);
        {
            world_to_odom_correction_ = initial_odom_transform.inverse();
            initial_rotation_captured_ = true;
        }
        return;
    }

    // Inject position and velocity into mujoco data
    Eigen::Vector3d current_odom_pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond current_odom_quat(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    // Current orientation and position in odom frame
    Eigen::Isometry3d current_odom_transform = Eigen::Isometry3d::Identity();
    current_odom_transform.translate(current_odom_pos);
    current_odom_transform.rotate(current_odom_quat);

    // Apply correction to get the position in the "world" frame
    Eigen::Isometry3d corrected_transform = world_to_odom_correction_ * current_odom_transform;

    // Corrected pos
    Eigen::Vector3d corrected_pos = corrected_transform.translation();

    // Corrected linear velocity
    Eigen::Vector3d odom_lin_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d corrected_lin_vel = world_to_odom_correction_.rotation() * odom_lin_vel;

    // Corrected orientation
    Eigen::Quaterniond corrected_quat = Eigen::Quaterniond(corrected_transform.rotation());

    // Corrected angular velocity
    Eigen::Vector3d odom_ang_vel(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    Eigen::Vector3d corrected_ang_vel = world_to_odom_correction_.rotation() * odom_ang_vel;

    OdometryData data;

    data.timestamp = get_timestamp();
    // Position
    data.base_pos[0] = corrected_pos[0];
    data.base_pos[1] = corrected_pos[1];
    data.base_pos[2] = corrected_pos[2];
    // Velocity
    data.base_lin_vel[0] = corrected_lin_vel[0];
    data.base_lin_vel[1] = corrected_lin_vel[1];
    data.base_lin_vel[2] = corrected_lin_vel[2];
    // Angular velocity
    data.base_ang_vel[0] = corrected_ang_vel.x();
    data.base_ang_vel[1] = corrected_ang_vel.y();
    data.base_ang_vel[2] = corrected_ang_vel.z();
    // Orientation
    data.base_quat[0] = corrected_quat.w();
    data.base_quat[1] = corrected_quat.x();
    data.base_quat[2] = corrected_quat.y();
    data.base_quat[3] = corrected_quat.z();

    // Lock mutex and write into buffer
    {
        std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);
        buffer_.push_back(data);

        if (buffer_.size() > SYNC_BUFFER_MAX_SIZE)
        {
            buffer_.pop_front();
        }
    } // Free mutex
}