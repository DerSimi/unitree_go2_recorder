# Unitree Go2 Recorder
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-blue)

Capture and replay accurate, timestamped Unitree Go2 data for analysis and visualization using MuJoCo — compatible with NumPy and ROS2 bags.

# ![Project Thumbnail](assets/thumbnail.png)

# ⭐ Key Features
- ROS2 bag compatible (tested on Humble)
- Synchronized recording across topics
- Per-step timestamps
- Live MuJoCo visualization
- NumPy (.npy) output format
- Supports Unitree SportStateMode, Vicon, and go2_odometry for base estimation

Supported base estimators
- [Unitree SportStateMode](https://support.unitree.com/home/en/developer/sports_services)
- Vicon (a camera based motion capture system).
- [go2_odometry](https://github.com/inria-paris-robotics-lab/go2_odometry)

> **Note**  
> Sport state mode is not available when the robot is in low state mode (e. g. if you want to record policy data). Vicon offers the highest accuracy.

# 💾 Extracted Data
The following data is stored:
Low cmd:
- q - motor positions (12)
- dq - motor velocities (12)
- tau - torques (12)
- kp position gains (12)
- kd derivative gains (12)

Low state:
- quaternion (4)
- gyroscope (3)
- d - motor positions (12)
- dq - motor velocities (12)
- tau_est - estimated torques (12)
- q_raw (12)
- dq_raw (12)

Base estimation:
- base position
- base velocity (not implemented for vicon)

# 💻 Installation
Don't forget to install the dependencies:

- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2): Only tested with Humble!
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [mujoco](https://github.com/google-deepmind/mujoco) (version 3.2.7)
- [cnpy](https://github.com/rogersce/cnpy)
- `sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libspdlog-dev`
- [gum](https://github.com/charmbracelet/gum)

> **Important**  
> You must build MuJoCo from source, it is not enough to install the precompiled version!

Make sure all dependencies are installed and available in your system path. Then build the project with:
```zsh
git clone https://github.com/DerSimi/unitree_go2_recorder && cd unitree_go2_recorder
mkdir build && cd build
cmake .. && make
```
Note, if you see any errors, source your unitree ros2 workspace, especially, source `setup.sh` and `install/setup.sh`.
```zsh
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/install/setup.sh
```
To start the recorder, run
```zsh
./setup.sh
```
in the project root (interactive), or run the built executable in `build`:
```sh
./go2_recorder --mode <high|vicon|go2odometry> --model <model_path> --storage <storage_path>
```
The recording is always stored within the output folder in the project root, so it suffices, just giving a name, like `test.npy`.

# Python Installation
If required, the created `.npy` file in the `storage` directory can be rendered using Python.
See `renderer/play.py` for an example of how to use the recording.

Install all Python dependencies using `uv` or a package manager of your liking:
```zsh
uv venv --python 3.12
source .venv/bin/activate
uv pip install -r requirements.txt
```

Then run the code in `renderer/play.py`:
```zsh
python renderer/play.py
```

# Usage
Start the interactive setup script:
```zsh
setup.sh
```
The script helps you checking your setup, including the network.

> **Note**  
> Setting up the vicon network requires you to change `setup.sh`.

At the top, insert your data:
```zsh
VICON_ETH_INTERFACE="enx34298f722bdf"
VICON_IP="10.0.0.20"
```

In the lab, I used a direct ethernet link to the vicon system. Make sure, the network interface for vicon is unmanaged, otherwise it may be removed by the network manager. 

# ROS2 Bags
The recorder also supports ROS2 bags. To record, run:
```zsh
ros2 bag record /lowstate /lowcmd /sportmodestate -o myrecording
```

Then start the Go2Recorder and play the bag:
```zsh
ros2 bag play myrecording
```
> **Note**  
> Make sure, that you use in both shells the same ROS domain.

You can test with the sample bag in the samples folder — unpack the archive first.

> **Important**  
> This applies only to SportStateMode and go2_odometry. Vicon does not publish ROS2 topics. For go2_odometry, record the topic: `/odometry/filtered`.

# Details on Synchronization and Timestamp Feature
You will notice that the recording contains a different number of messages for each relevant topic. This requires a synchronization strategy. I chose to use the topic with the most messages, which is `/lowcmd`, as the reference. The program always takes the oldest `/lowcmd` message in the buffer and searches for the closest (but not newer) messages from the other topics. 

Note that the timestamps here are based on the (rather inaccurate) PC time, since there are no timestamps available in `/lowcmd` and `/lowstate`.

# Credits
This project is based on a heavily rewritten version of the work from [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco). The XML model is also adapted from that repository.
