# a2_unitree_bridge

ROS 2 bridge between the Unitree DDS layer and ROS 2 topics for the A2 robot. Two build targets share the same source — one for the real robot (HG firmware) and one for MuJoCo simulation.

## Build

```bash
colcon build --packages-select a2_unitree_bridge
```

## Run

**Simulation (MuJoCo + Unitree DDS on loopback):**
```bash
ros2 launch a2_unitree_bridge sim.launch.py
```

**Real robot (ethernet, HG firmware):**
```bash
ros2 launch a2_unitree_bridge robot.launch.py
```

## Topics

### Sim (`a2_bridge_sim`)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | DDS rate | Sim time derived from MuJoCo `tick` |
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Throttled from LowState DDS |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | Throttled from LowState DDS |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Ground-truth pose from SportModeState |
| `/state_estimation` | `nav_msgs/Odometry` | 50 Hz | Same as `/odom` (for downstream compatibility) |
| `/a2/sport_mode` | `std_msgs/UInt8` | 50 Hz | Current sport mode enum value |
| `/camera/image_raw` | `sensor_msgs/Image` | DDS rate | Front camera RGB decoded from PointCloud2 DDS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | DDS rate | Intrinsics from `a2_description/config/camera_info_sim.yaml`, published with each image |
| `/front_lidar/points` | `sensor_msgs/PointCloud2` | DDS rate | Raw front lidar scan |
| `/rear_lidar/points` | `sensor_msgs/PointCloud2` | DDS rate | Raw rear lidar scan |
| `/lowcmd` (subscribed) | `unitree_go/LowCmd` | — | Receives low-level joint commands and forwards to DDS |

**TF:** `map → base_link` broadcast at 50 Hz from SportModeState ground-truth position.

### Robot (`a2_bridge_robot`)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Throttled from LowState DDS (HG firmware) |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | Throttled from LowState DDS |
| `/a2/sport_mode` | `std_msgs/UInt8` | 50 Hz | Current sport mode enum value |
| `/a2/mode` (subscribed) | `a2_interfaces/msg/OperatingMode` | — | Requests FSM mode transitions |
| `/cmd_vel` (subscribed) | `geometry_msgs/TwistStamped` | — | Velocity commands; must be stamped, dropped if >500 ms old |

Odometry on the real robot comes from DLIO, not this bridge.

## Operating the robot (`a2_bridge_robot`)

The robot is driven through a mode state machine (FSM) defined in `a2_utils`. Modes must be stepped through in order — you cannot jump directly from `STAND_DOWN` to `VELOCITY_MOVE`.

### Mode transitions

Publish to `/a2/mode` (`a2_interfaces/msg/OperatingMode`, field `mode: uint8`) to request a transition:

| Value | Mode | Description |
|---|---|---|
| 0 | `ESTOP` | Damp all joints immediately. Reachable from any mode. |
| 1 | `STAND_DOWN` | Sit down, joints locked. Starting state on launch. |
| 2 | `STAND_UP` | Stand up from `STAND_DOWN`. |
| 3 | `BALANCE_STAND` | Active balance stand from `STAND_UP`. |
| 4 | `VELOCITY_MOVE` | Accept velocity commands. Reachable from `BALANCE_STAND`. |
| 5 | `FREE` | Stop motion without disabling joints. Reachable from any mode. |

Valid transition sequence to start moving:
```
STAND_DOWN → STAND_UP → BALANCE_STAND → VELOCITY_MOVE
```

From `FREE`, the robot can return to whatever mode it was in before `FREE` was requested, or transition to `STAND_DOWN`.

Example (CLI):
```bash
ros2 topic pub --once /a2/mode a2_interfaces/msg/OperatingMode "{mode: 1}"  # STAND_DOWN
ros2 topic pub --once /a2/mode a2_interfaces/msg/OperatingMode "{mode: 2}"  # STAND_UP
ros2 topic pub --once /a2/mode a2_interfaces/msg/OperatingMode "{mode: 3}"  # BALANCE_STAND
ros2 topic pub --once /a2/mode a2_interfaces/msg/OperatingMode "{mode: 4}"  # VELOCITY_MOVE
```

### Velocity commands

Once in `VELOCITY_MOVE`, publish `geometry_msgs/TwistStamped` to `/cmd_vel`. Commands older than 500 ms (by header stamp) are dropped, and velocity is zeroed after each control tick — the publisher must send continuously at the desired rate.

| Field | Axis | Limit |
|---|---|---|
| `twist.linear.x` | Forward/back | ±0.15 m/s |
| `twist.linear.y` | Left/right | ±0.10 m/s |
| `twist.angular.z` | Yaw rate | ±0.10 rad/s |

The control loop runs at 50 Hz.
