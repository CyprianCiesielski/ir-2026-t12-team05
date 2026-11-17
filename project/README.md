# Final Project - The Phoenyx-I Mars Mission

Intelligent Robotics 2026 (UPC-EEBE) – T12 Team 05.

## Project Description

Phoenyx-I must cross a damaged Martian research station autonomously and reach the Mars Return Base.

- **Level 1 – Service Corridor (required):** stay centred in narrow corridors and complete 90-degree turns using a PID controller.
- **Level 2 – Rocky Sector (required):** detect rocks with the LiDAR, avoid them and recover corridor navigation.
- **Level 3 – Advanced Mission (optional):** 3.A Lost Sample Recovery or 3.B Control Panel Inspection. *Selected mission: to be decided.*

## Repository Structure

```text
project
├── README.md
├── start_container.sh
├── enter_container.sh
├── osr_gazebo                     # Phoenyx-I model (URDF, meshes, controller) – provided
│   ├── launch
│   ├── rviz
│   ├── src/osr_controller.cpp
│   └── urdf
├── phoenyx_i_bringup              # Mission launch, RViz configuration and worlds
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch
│   │   └── sim.launch.py
│   ├── rviz
│   │   └── phoenyx_i.rviz
│   └── worlds
│       ├── maze_1.world
│       ├── maze_2.world
│       └── maze_3.world
└── phoenyx_i_control              # Autonomous control nodes
    ├── package.xml
    ├── setup.py
    └── phoenyx_i_control
        └── corridor_controller.py
```

## Robot Configuration

| Sensor | Topic | Message |
|--------|-------|---------|
| LiDAR (`laser_frame`) | `/scan` | `sensor_msgs/LaserScan` |
| IMU (`imu_link`) | `/imu_plugin/out` | `sensor_msgs/Imu` |

The rover is commanded with `geometry_msgs/Twist` on `/cmd_vel`.

## System Architecture

```text
Gazebo ──/scan (LaserScan)───────▶ corridor_controller ──/cmd_vel (Twist)──▶ osr_controller ──▶ wheel / servo controllers
```

## Level 1 – Corridor Controller

`corridor_controller` runs on every `/scan` message and uses only the LiDAR.

**Perception.** The scan is reduced to three distances (minimum valid range in each sector), as in Session 6: left (30°–90°), front (−15°–15°) and right (−90°–−30°).

**Straight sections – PID.** The navigation error is `e = min(left, 0.8) − min(right, 0.8)`. Capping both sides at 0.8 m means an open side (a corridor opening) does not pull the rover into it. A positive error means the rover is too close to the right wall, so it turns left:

`angular.z = Kp·e + Ki·Σe + Kd·Δe` with `Kp = 1.0`, `Ki = 0`, `Kd = 2.0`, `linear.x = 0.2 m/s`.

`angular.z` is limited to `0.2 / 0.6` rad/s, because `osr_controller` converts `v/ω` into a turning radius and cannot steer tighter than ~0.18 m.

**Corners.** When the front distance drops below 0.7 m the rover rotates in place (`linear.x = 0`, `angular.z = ±0.5`) towards the side with more space, until the front distance is above 1.5 m again. The same command is sent during the whole turn, so the steering servos move only once. A dead end is handled the same way: the rover keeps rotating until it faces a free path.

## Execution Instructions

All commands run inside the Docker container.

```bash
# Build
cd ~/ir_ws
colcon build
source install/setup.bash

# Terminal 1 – Simulation (maze_1.world, maze_2.world or maze_3.world)
ros2 launch phoenyx_i_bringup sim.launch.py world:=maze_1.world

# Terminal 2 – Teleoperation (stop it before running the controller)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 2 – Autonomous controller
ros2 run phoenyx_i_control corridor_controller
```

## Progress

### Session 1 – Prepare the Mission Environment
- Downloaded `osr_gazebo` and removed its `worlds` directory (also removed from its `CMakeLists.txt` install rule).
- Created `phoenyx_i_bringup` with the Level 1 worlds, a custom RViz configuration (RobotModel, TF, LaserScan) and `sim.launch.py` with a `world` launch argument.
- Created `phoenyx_i_control` with the `corridor_controller` node structure.

### Session 2 – Level 1 Corridor Controller
- Implemented the PID corridor-following controller with LiDAR-based corner turns.
- First version used the IMU and a state machine; in Gazebo the rover kept re-positioning its servos and made small rotations instead of 90° turns, so it was simplified to a constant-speed rotation until the front is free.
- The `/scan` subscription uses `qos_profile_sensor_data` (best-effort), which is compatible with the Gazebo sensor plugin.

## Results

To be completed.

## Conclusions

To be completed.
