#!/usr/bin/env zsh
VICON_ETH_INTERFACE="enx34298f722bdf"
VICON_IP="10.0.0.20"

ROBOT_IP="192.168.123.99"

wait_animation() {
    local msg="$1"
    gum spin --spinner dot --title "$msg" -- sleep 2
    echo "${PREFIX} $msg"
}

ping_test() {
    local ip="$1"
    if ping -c 1 -W 1 $ip &> /dev/null; then
        return 0
    else
        return 1
    fi
}

setup_ros2() {
    local robot="$1"
    if [ "$robot" -eq 1 ]; then
        source $GO2_ROS/setup.sh
    else
        source $GO2_ROS/setup_local.sh
    fi
    source $GO2_ROS/install/setup.sh

}

check_ros2() {
    ros2 topic list 2> /dev/null | grep -q "/lowstate"
}

start_program() {
    if [ ! -f build/go2_recorder ]; then
        echo "${PREFIX} Build target not found, building first..."
        mkdir -p build
        cd build
        cmake ..
        make
        cd ..
    fi
    echo "${PREFIX} starting the Go2Recorder..."
    cd build
    ./go2_recorder "$@"
}

if ! command -v gum &> /dev/null; then
    echo "gum is required for this script, please install it: https://github.com/charmbracelet/gum"
    exit 1
fi

if [ -z "$GO2_ROS" ]; then
    echo "${PREFIX} No GO2_ROS environment variable set. Please install ros2 for unitree and set the GO2_ROS variable accordingly. \n See: https://github.com/unitreerobotics/unitree_ros2"
    exit 1
fi

arg_count=$#

# Setup logger
PREFIX="\033[38;5;250m[\033[0m\033[38;5;45mGo2 Recorder\033[0m\033[38;5;250m]\033[0m"

# When the user enters arguments, he want's to directly use the program, without an interactive intro
manual_mode=false
if [ $arg_count -ge 2 ] && [ $arg_count -le 7 ]; then
    manual_mode=true
fi

if [ "$manual_mode" = true ]; then
    echo "${PREFIX} Manual mode detected, skipping interactive setup. Note, no network checks will be performed."
    
    if [ "$1" = "robot" ]; then
        setup_ros2 1
    else
        setup_ros2 0
    fi
    
    shift # remove first argument
    start_program "$@"
    exit 0
fi

echo "${PREFIX} This interactive script will help you getting the right command and everything else around setup for data collection."

mode=""
if [ "$manual_mode" = false ]; then
    mode=$(gum choose "Simulator data" "Real world data" --header="What data do you want to collect?")
    echo "${PREFIX} You selected: $mode"
fi

estimator=""
if [ "$mode" = "Real world data" ]; then
    echo "${PREFIX} Data collection in the real world requires selecting a base state estimator."
    estimator=$(gum choose "SportModeState" "Vicon" "Go2Odometry" --header="Select the base estimator")
fi

if [ "$estimator" = "SportModeState" ]; then
    estimator="high"
    echo "${PREFIX} You selected SportModeState as base estimator. This is a ros topic published by the Go2 robot, but not available in the low state mode which is used when playing RL-policies."
elif [ "$estimator" = "Vicon" ]; then
    estimator="vicon"
    echo "${PREFIX} You selected Vicon as base estimator. Make sure to setup the system, and configure the network."
elif [ "$estimator" = "Go2Odometry" ]; then
    estimator="go2odometry"
    echo "${PREFIX} You selected Go2Odometry as base estimator. This is also available in low state mode, but less accurate than the other two options. See: https://github.com/inria-paris-robotics-lab/go2_odometry"
fi

if [ "$estimator" = "" ]; then
    echo "${PREFIX} Since you are collecting simulator data, the perfect MuJoCo ground truth will be used as base estimator."
fi

wait_animation "Setting up ROS2 in $GO2_ROS..."
echo " "

if [ "$mode" = "Real world data" ]; then
    setup_ros2 1
elif [ "$mode" = "Simulator data" ]; then
    estimator="high"
    setup_ros2 0
fi

echo " "

wait_animation "If the ros setup fails, terminate the script at this point!"

if [ "$mode" = "Real world data" ]; then
    wait_animation "We now continue with the network setup. Pinging the robot..."

    # This part checks the robot and if ros works
    echo "${PREFIX} Pinging the robot at $ROBOT_IP ..."
    if ping_test $ROBOT_IP; then
        echo "${PREFIX} Robot is reachable."
    else
        echo "${PREFIX} Robot is NOT reachable. Please check your network settings. \n Also see setup instructions here: https://github.com/unitreerobotics/unitree_ros2"
        exit 1
    fi

    echo "${PREFIX} Checking if ROS2 is working... If this takes too long, you know what the problem is. :)"
    if check_ros2; then
        echo "${PREFIX} ROS2 is working and ready to use."
    else
        echo "${PREFIX} The ros setup is not working. Try restarting everything, including the robot."
        exit 1
    fi

    wait_animation "Checking if the ros setup is working..."

    # In here, we check the vicon system.
    if [ "$estimator" = "vicon" ]; then
        echo " "
        wait_animation "Pinging the Vicon server at $VICON_IP ..."

        if ping_test $VICON_IP; then
            echo "${PREFIX} Vicon server is reachable."
        else
            echo "${PREFIX} Vicon server is NOT reachable. We will attempt to set up the network interface $VICON_ETH_INTERFACE now. Make sure this network adapter is unmanaged by your network manager."
            echo "${PREFIX} For this purpose, I suggest:"
            echo "  sudo ip addr del $VICON_IP/24 dev $VICON_ETH_INTERFACE 2>/dev/null"
            echo "  sudo ip addr add $VICON_IP/24 dev $VICON_ETH_INTERFACE"
            echo "  sudo ip link set $VICON_ETH_INTERFACE up"

            if ! gum confirm "Do you agree?"; then
                echo "${PREFIX} Consider changing the vicon ip and interface name in this script."
                exit 1
            fi

            # This may ask for a password...
            sudo ip addr del $VICON_IP/24 dev $VICON_ETH_INTERFACE 2>/dev/null
            sudo ip addr add $VICON_IP/24 dev $VICON_ETH_INTERFACE

            sudo ip link set $VICON_ETH_INTERFACE up

            # if vicon fails after setting up the network, ...
            if ! ping_test $VICON_IP; then
                echo "${PREFIX} Vicon server is NOT reachable, setup failed. Please check your vicon and network setup."
                exit 1
            fi
        fi
    fi
fi

# Select model
echo "${PREFIX} The most part is done. Now, you can choose the MuJoCo model to use, for this see the '/model' directory."
model=$(gum input --placeholder "What model do you want to use?" --value "model/scene.xml")
echo "${PREFIX} Selected model: $model"

echo "${PREFIX} And at last, provide a name for the data recording."
data_name=$(gum input --placeholder "How should we call it? Please don't use spaces :/" --value "test.npy")
echo "${PREFIX} Data will be recorded to: output/$data_name."


if [ "$mode" = "Real world data" ]; then
    mode="robot"
elif [ "$mode" = "Simulator data" ]; then
    mode="sim"
fi

echo "${PREFIX} Starting the data collection now... \n To skip the tutorial the next time, use this command: \n ./extractor.sh $mode --mode $estimator --model $model --storage $data_name"

start_program --mode $estimator --model $model --storage $data_name