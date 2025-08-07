#ifndef UNITREE_SDK2_BRIDGE_H
#define UNITREE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <mujoco/mujoco.h>

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_WIRELESS_CONTROLLER "rt/wirelesscontroller"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20
#define NUM_MOTOR_IDL_HG 35

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);
    ~UnitreeSdk2Bridge();

    void LowCmdGoHandler(const void *msg);
    void LowCmdHgHandler(const void *msg);

    void PublishLowStateGo();
    void PublishLowStateHg();
    void PublishHighState();
    void PublishWirelessController();
    void Run();
    void PrintSceneInformation();
    void CheckSensor();
    void SetupJoystick(string device, string js_type, int bits);

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> low_cmd_hg_suber_;

    unitree_go::msg::dds_::LowState_ low_state_go_{};
    unitree_hg::msg::dds_::LowState_ low_state_hg_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    unitree_go::msg::dds_::WirelessController_ wireless_controller_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> low_state_hg_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> high_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> wireless_controller_puber_;

    ThreadPtr lowStatePuberThreadPtr;
    ThreadPtr HighStatePuberThreadPtr;
    ThreadPtr WirelessControllerPuberThreadPtr;

    int max_value_ = (1 << 15); // 16 bits joystick

    mjData *mj_data_;
    mjModel *mj_model_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
    int idl_type_ = 0; // 0: unitree_go, 1: unitree_hg
};

#endif
