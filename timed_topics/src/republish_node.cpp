#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "unitree_go/msg/low_state.hpp"
#include "timed_topics/msg/timed_low_state.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "timed_topics/msg/timed_low_cmd.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include "timed_topics/msg/timed_sport_mode_state.hpp"

using std::placeholders::_1;

class RepublishNode : public rclcpp::Node {
public:
  RepublishNode()
  : Node("timed_republisher")
  {
    pub_low_state_ = this->create_publisher<timed_topics::msg::TimedLowState>("timedlowstate", 10);
    sub_low_state_ = this->create_subscription<unitree_go::msg::LowState>(
      "lowstate", 10,
      std::bind(&RepublishNode::on_low_state, this, _1));

    pub_low_cmd_ = this->create_publisher<timed_topics::msg::TimedLowCmd>("timedlowcmd", 10);
    sub_low_cmd_ = this->create_subscription<unitree_go::msg::LowCmd>(
      "lowcmd", 10,
      std::bind(&RepublishNode::on_low_cmd, this, _1));

    pub_sport_ = this->create_publisher<timed_topics::msg::TimedSportModeState>("timedsportmodestate", 10);
    sub_sport_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      std::bind(&RepublishNode::on_sport_state, this, _1));

    RCLCPP_INFO(this->get_logger(), "Republisher: Ready to LowState/LowCmd/SportModeState -> Timed msgs");
  }

private:
  void on_low_state(const unitree_go::msg::LowState::SharedPtr msg) {
    timed_topics::msg::TimedLowState out;
    uint64_t ns = this->now().nanoseconds();
    out.stamp.sec = ns / 1000000000ULL;
    out.stamp.nanosec = ns % 1000000000ULL;
    out.state = *msg; 
    pub_low_state_->publish(out);
  }

  void on_low_cmd(const unitree_go::msg::LowCmd::SharedPtr msg) {
    timed_topics::msg::TimedLowCmd out;
    uint64_t ns = this->now().nanoseconds();
    out.stamp.sec = ns / 1000000000ULL;
    out.stamp.nanosec = ns % 1000000000ULL;
    out.state = *msg;
    pub_low_cmd_->publish(out);
  }

  void on_sport_state(const unitree_go::msg::SportModeState::SharedPtr msg) {
    timed_topics::msg::TimedSportModeState out;
    uint64_t ns = this->now().nanoseconds();
    out.stamp.sec = ns / 1000000000ULL;
    out.stamp.nanosec = ns % 1000000000ULL;
    out.state = *msg;
    pub_sport_->publish(out);
  }

  rclcpp::Publisher<timed_topics::msg::TimedLowState>::SharedPtr pub_low_state_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_low_state_;

  rclcpp::Publisher<timed_topics::msg::TimedLowCmd>::SharedPtr pub_low_cmd_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr sub_low_cmd_;

  rclcpp::Publisher<timed_topics::msg::TimedSportModeState>::SharedPtr pub_sport_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_sport_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepublishNode>());
  rclcpp::shutdown();
  return 0;
}