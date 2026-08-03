# Unitree Go2 Recorder
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-blue)

Capture and replay accurate, high frequency timestamped Unitree Go2 data for analysis and visualization using MuJoCo — compatible with NumPy and ROS2 bags. Topics are matched and merged with different data sources, like Vicon for base estimation.

> **Note**  
> Data is only recorded when commands are received, either from the remote control or via ROS topic.

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

> **Note**  
> Sport state mode is not available when the robot is in low state mode (e. g. if you want to record policy data). Vicon offers the highest accuracy. For vicon, you find more information in the `docs` folder.

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

# 🧩 Dependencies
Install the following dependencies:
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2): Only tested with Humble!
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [mujoco](https://github.com/google-deepmind/mujoco) (version 3.2.7)
- [cnpy](https://github.com/rogersce/cnpy)
- `sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libspdlog-dev`
- [gum](https://github.com/charmbracelet/gum)
- [Vicon](https://github.com/intelligent-soft-robots/vicon-datastream-sdk)

> **Important**  
> You must build MuJoCo from source, it is not enough to install the precompiled version!

# 📦 Source ROS environment
Before compiling anything, make sure you source the `setup.sh` from [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2). Also source the setup script in its installation folder.
```zsh
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/install/setup.sh
```

# 🛠️ Timed Topics
For accurate time measurement, install and set up the `timed_topics` ROS package on the Unitree Go2's internal Nvidia Jetson. You only need [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2), but be sure to follow the steps from the previous section.

```zsh
cd timed_topics
colcon build --packages-select timed_topics
```

You also need to build the `timed_topics` package on your workstation for building the Go2Recorder itself, see next capture.

Start the node only on the Nvidia Jetson:
```zsh
ros2 run timed_topics republish_node 
```
Alternatively, you can start it on your workstation, but timestamp accuracy may be affected by system load. In this case, you should extend your
python environment to also use the system packages.
```zsh
python3.10 -m venv .venv --system-site-packages
uv pip install catkin_pkg "empy<4" lark
```
This is confusing at this point, continue reading, until capture `Python Installation`.

> **Important**  
> For time synchronization, especially when using Vicon, synchronize the Nvidia Jetson and your workstation. I used [chrony](https://chrony-project.org/) for this.

# 🛠️ Build Go2Recorder
Make sure all dependencies are installed and available in your system path.
Then build the project with:
```zsh
mkdir build && cd build
cmake .. && make
```
> ** Note** 
> If you see any errors, make sure your ROS environment is properly sourced. The `timed_topics` package must be compiled and sourced first.

Set the path to your Unitree ROS2 workspace by adding the following to your shell startup file (e.g. `~/.zshrc` or `~/.bashrc`):
```zsh
export GO2_ROS="$HOME/unitree_ros2"
```

Reload your shell configuration:
```zsh
source ~/.zshrc # or: source ~/.bashrc
```

To start the recorder, run
```zsh
./setup.sh
```
in the project root (interactive), or run the built executable in `build`:
```sh
./go2_recorder --mode <high|vicon> --model <model_path> --storage <storage_path>
```
The recording is always stored within the output folder in the project root, a simple name suffices, e. g., `test.npy`.

> **Note**  
> When recording simulation data only, the recorder expects [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco) as virtual robot. This is already part of this repo as a submodule. Build it, start ./mujoco.sh and create a ROS node which sends commands to the environment.

# 💻 Python Installation
If required, the created `.npy` file in the `storage` directory can be rendered using Python.
See `renderer/play.py` for an example of how to use the recording.

Install all Python dependencies using `uv` or a package manager of your liking:
```zsh
uv sync
```

And don't forget to activate the environment:
```zsh
source .venv/bin/activate
```

For timestamp inspection run:
```zsh
python debugging/latency.py
```

To create a video from your recording, use:
```zsh
python debugging/render.py
```

# Usage
Start the interactive setup script:
```zsh
setup.sh
```
The script helps you checking your setup, including the network.

To save your recording, just close the MuJoCo window or press Ctrl+C in the terminal.

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

> **Important**  
> This applies only to SportStateMode. Vicon does not publish ROS2 topics.

# Details on Synchronization and Timestamp Feature
You will notice that the recording contains a different number of messages for each relevant topic. This requires a synchronization strategy. I chose to use the topic with the most messages, which is `/lowstate`, as the reference. The program always takes the closest `/lowcmd` message in the buffer and searches for the closest messages from the other topics. For Vicon data, interpolation is used: the algorithm finds the two nearest measurements (one older, one newer) and interpolates position, velocity, and rotation (using slerp for orientation).

Note that the timestamps here are based on the (rather inaccurate) PC time, since there are no timestamps available in `/lowcmd` and `/lowstate`.

# Credits
This project is based on a heavily rewritten version of the work from [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco). The XML model is also adapted from that repository.
