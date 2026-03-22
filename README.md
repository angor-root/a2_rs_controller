# a2_ros

ROS2 (Jazzy) simulation of the Unitree A2 quadruped using MuJoCo and a trained RL locomotion policy.

## Setup

**1. Install Unitree SDK2** (manual step — installs to `/opt/unitree_robotics/`):
https://github.com/unitreerobotics/unitree_sdk2

**2. Clone and run the install script** (handles MuJoCo, system packages, Python deps, and builds the workspace):
```bash
conda deactivate   # must use system Python, not conda
git clone git@github.com:ETHZ-RobotX/a2_ros.git --recursive
bash a2_ros/install.sh
```

## Run

Source the environment in every terminal:
```bash
source a2_ros/setup.sh
```

Launch the full simulation:
```bash
ros2 launch a2_ros sim.launch.py
ros2 launch a2_ros sim.launch.py rviz:=true
ros2 launch a2_ros sim.launch.py scene:=scene_terrain.xml
```

## Gamepad

| Input | Action |
|---|---|
| Left stick | Forward / strafe |
| Right stick horizontal | Yaw |
| X + L2 | Sit |
| Triangle + L2 | Stand |
| L2 + R2 | Walk |
