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
Gazebo ──/imu_plugin/out (Imu)───▶ corridor_controller
```

## Level 1 – Corridor Controller

`corridor_controller` is a state machine driven by every `/scan` message:

| State | Behaviour |
|-------|-----------|
| `FOLLOW` | Drive forward and keep the rover centred with a PID controller. Slow down when the front wall is closer than 1.5 m. |
| `PAUSE` | Stop for 0.6 s so the steering servos can reach their new angles. |
| `TURN` | Rotate in place (`linear.x = 0`) until the IMU yaw reaches the target heading (±90°, or 180° at a dead end). |

**Perception.** The LiDAR is reduced to three distances (minimum valid range in each sector): left (80°–100°), front (−15°–15°) and right (−100°–−80°). Returns closer than 0.25 m are ignored as self-hits.

**Navigation error.** Positive error means the rover is too close to the right wall:
- both walls visible → `e = (left − right) / 2`
- only one wall visible → distance to that wall minus 0.75 m (half the corridor width)
- no walls → `e = 0`

**Control law.** `ω = Kp·e + Ki·∫e + Kd·de/dt + Kh·(yaw_target − yaw)` with `Kp = 1.5`, `Ki = 0.02`, `Kd = 0.5`, `Kh = 0.5`. The heading term (IMU) keeps the rover parallel to the corridor. `ω` is limited to `v / 0.6 m` because `osr_controller` converts `v/ω` into a turning radius and cannot steer tighter than ~0.18 m.

**Corners.** When the front distance drops below 0.7 m the rover stops and turns towards the open side (a side farther than 1.5 m). If both sides are closed it turns around.

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
- Implemented the PID corridor-following controller with IMU-based 90° turns and dead-end handling.
- Subscriptions use `qos_profile_sensor_data` because the Gazebo sensor plugins publish with best-effort QoS.

## Results

To be completed.

## Conclusions

To be completed.
