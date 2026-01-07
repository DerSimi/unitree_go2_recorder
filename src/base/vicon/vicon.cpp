#include "base/vicon/vicon.hpp"

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
    // Unfortunately, no good vicon timestamp is available.
    // We use system time instead, danger of jitter, make sure clocks between robot and workstation are in sync.
    data.stamp = rclcpp::Clock().now();

    for (int i = 0; i < 3; i++)
        data.base_pos[i] = position[i] / 1000.0; // Convert mm to m
    for (int i = 0; i < 4; i++)
        data.orientation[i] = orientation[i];

    // TODO: fix this
    // Calculate linear velocity
    for (int i = 0; i < 3; i++)
        data.world_lin_vel[i] = 0.0;

    // TODO: fix velocity calculation
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

    // last_timestamp_ = data.timestamp;
    // std::copy(position.begin(), position.end(), last_position_.begin());

    std::lock_guard<std::mutex> lock(DataSourceBase::sync_mtx_);
    buffer_.push_back(data);

    if (buffer_.size() > SYNC_BUFFER_MAX_SIZE)
    {
        buffer_.pop_front();
    }
} // Free mutex

bool ViconDataSource::get_closest_match(rclcpp::Time &time, void *res)
{
    ViconData *vicon_res = static_cast<ViconData *>(res);

    if (buffer_.size() < 2)
        return false;

    // Look for the first element that is older then the stamp.
    // Go from newest (back) to oldest (front)
    ViconData old;
    ViconData new_;

    bool found = false;
    std::deque<ViconData>::reverse_iterator old_it;

    for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it)
    {
        // the first buffer element which is older...
        if (it->stamp.nanoseconds() < time.nanoseconds())
        {
            old = *it;
            old_it = it;
            found = true;
            break;
        }
    }

    if (!found)
        return false;

    auto future_it = old_it.base();

    if (future_it == buffer_.end())
        return false;

    new_ = *future_it;

    if (!(old.stamp.nanoseconds() < time.nanoseconds() && time.nanoseconds() <= new_.stamp.nanoseconds()))
        return false;

    // Now interpolate position
    auto t_1 = old.stamp;
    auto t_2 = new_.stamp;

    // Interpolate position
    for (int i = 0; i < 3; i++)
    {
        mjtNum s_1 = old.base_pos[i];
        mjtNum s_2 = new_.base_pos[i];

        vicon_res->base_pos[i] = interpolate_time(t_1, t_2, s_1, s_2, time);

        mjtNum v_1 = old.world_lin_vel[i];
        mjtNum v_2 = new_.world_lin_vel[i];

        vicon_res->world_lin_vel[i] = interpolate_time(t_1, t_2, v_1, v_2, time);
    }

    // Interpolate orientation
    Eigen::Quaterniond rot_1(old.orientation);
    Eigen::Quaterniond rot_2(new_.orientation);

    Eigen::Quaterniond result;
    interpolate_quat(t_1, t_2, rot_1, rot_2, time, result);

    // Write back
    vicon_res->orientation[0] = result.x();
    vicon_res->orientation[1] = result.y();
    vicon_res->orientation[2] = result.z();
    vicon_res->orientation[3] = result.w();

    return true;
}