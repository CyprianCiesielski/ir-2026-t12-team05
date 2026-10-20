# Intelligent Robotics - Johny

Intelligent Robotics 2026 (UPC-EEBE) – T12 Team 05. Practice sessions and final project.



## Team Information

- **Team members:** Cyprian Ciesielski, Laila Fahchouch

- **Practice group:** T12

- **Team number:** Team05

- **Robot name:** Johny



## Repository Structure

```text

ir-2026-t12-team05

├── README.md

├── session1

│   ├── robot_startup.sh

│   └── team_info.txt

├── session2

│   └── johny.urdf

├── session3

│   ├── johny_brain.py

│   └── johny_description

│       ├── CMakeLists.txt

│       ├── package.xml

│       ├── launch

│       │   └── display.launch.py

│       ├── rviz

│       │   └── urdf_config.rviz

│       └── urdf

│           └── johny.urdf

└── session4

    ├── enter_container.sh

    ├── start_container.sh

    └── johny_bringup

        ├── CMakeLists.txt

        ├── package.xml

        ├── launch

        │   └── johny.launch.py

        ├── rviz

        │   └── urdf_config.rviz

        └── urdf

            └── johny.urdf

```



## Practice Sessions



### Session 1 - Get Familiar with the Tools



#### Overview

In this session, we set up the core development environment and version control workflow for the Intelligent Robotics course:

- **Linux Terminal & Workspace:** Explored essential terminal commands (`cd`, `mkdir`, `touch`, `cp`, `mv`, `rm`, `cat`, redirections) and organized the course workspace directory structure (`~/ir_ws/src`).

- **Bash Scripting:** Developed `robot_startup.sh` with a proper shebang (`#!/bin/bash`), which loads team information from `team_info.txt`, displays an ASCII art welcome banner, outputs the absolute script path, and lists the session files. Configured execution permissions using `chmod +x`.

- **Git & GitHub Integration:** Created a private team repository on GitHub, configured secure authentication via Ed25519 SSH keys, cloned the repository locally into the workspace, and practiced the complete Git lifecycle (`status`, `add`, `commit`, `push`).



#### How to Run

To run the robot startup script, execute the following commands from the root of the repository:



```bash

chmod +x session1/robot_startup.sh

./session1/robot_startup.sh

```



### Session 2 - Shape Your Robot in Simulation



#### Overview

In this session, we designed and built our first digital robot model using the Unified Robot Description Format (URDF) and visualized it in 3D using RViz2:

- **Git Warm-Up:** Synchronized local and remote changes using `git fetch`, `git status`, and `git pull` prior to development.

- **URDF Fundamentals:** Learned how to describe a robot's mechanical structure using XML syntax, specifying rigid bodies (`<link>`) with visual geometries (box, cylinder, sphere) and defining interconnections (`<joint>`).

- **Differential-Drive Modeling:**

  - Built the main chassis (`base_link`) with defined dimensions and materials.

  - Added two active driving wheels (`right_wheel_link` and `left_wheel_link`) using `continuous` joints to allow endless rotation along the appropriate axis.

  - Added a front caster wheel modelled as a spherical link with a `fixed` joint to provide passive balance without sinking.

- **Ground Alignment (`base_footprint`):** Created a zero-inertia, invisible `base_footprint` root link connected via `base_joint` to align the coordinate frame directly with the ground plane, ensuring wheels rest accurately on the grid.



#### How to Run

To visualize the URDF robot model in RViz2, navigate to the `session2` folder and launch the display tool:



```bash

cd session2

ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/johny.urdf

```



### Session 3 - Build the Foundations of Your Robot



#### Overview

In this session, we developed an autonomous controller algorithm in Python and integrated our robot model into a modular ROS 2 description package:

- **Part 1 – Robot Brain (Robot Run Challenge):** Developed `johny_brain.py` to parse simulated ultrasonic sensor distances (`LEFT`, `FRONT`, `RIGHT` from `scan.ranges`) and implemented an obstacle avoidance algorithm that autonomously commands the robot (`FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `STOP`).

- **Part 2 – ROS 2 Description Package:**

  - Created the `johny_description` package, cleanly removing unneeded C++ boilerplate and configuring `CMakeLists.txt` to install `urdf/`, `launch/`, and `rviz/` resource directories.

  - Authored a Python launch file (`display.launch.py`) that loads the URDF description via `xacro` and launches `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

  - Automated visualization by pre-configuring RViz2 (setting `base_footprint` as Fixed Frame, enabling `RobotModel` and `TF`) and saving the configuration to `rviz/urdf_config.rviz` for seamless future launches.



#### How to Run



1. **Run the Autonomous Brain (Robot Run arcade):**

```bash

cd session3

python3 johny_brain.py

```



2. **Build the Workspace and Launch in ROS 2 / RViz2:**

```bash

cd ~/ir_ws

colcon build

source ~/.bashrc

ros2 launch johny_description display.launch.py

```



### Session 4 - Bring Your Robot to Life



#### Overview

In this session, we introduced Docker for environment isolation and transitioned our robot from a purely visual model into a dynamic physical simulation using Gazebo:

- **Dockerization:** Configured and utilized Docker containers (`intelligent-robotics:humble`) using `start_container.sh` to ensure a consistent and isolated ROS 2 workspace without cross-team network interference. Created `enter_container.sh` to attach additional interactive terminals to the running container.

- **Physics & Collisions:** Upgraded the URDF by adding `<inertial>` properties (mass, origin, inertia matrix) and `<collision>` geometries to the base, drive wheels, and caster wheel.

- **Gazebo Integration:** 

  - Assigned native Gazebo materials/colors (`Gazebo/Blue`, etc.) using the `<gazebo>` tag.

  - Adjusted physical interaction properties (friction coefficients `mu1` and `mu2` for the caster wheel) for smooth ground interactions.

  - Integrated the `libgazebo_ros_diff_drive.so` plugin to enable differential-drive control over the robot's wheels.

- **Simulation Bringup:** Created the `johny_bringup` package and wrote `johny.launch.py` to simultaneously start Gazebo, spawn the robot entity using `gazebo_ros`, publish robot state, and open RViz2.



#### How to Run



1. **Start the Docker Container:**

```bash

# In your host machine terminal:

cd session4

./start_container.sh

```



2. **Build the Workspace and Launch the Simulation:**

```bash

# Inside the Docker container:

cd ~/ir_ws

colcon build

source ~/.bashrc

ros2 launch johny_bringup johny.launch.py

```



3. **Control the Robot (Teleoperation):**

Open a new terminal, enter the running container, and start the keyboard teleoperation node:

```bash

# In a new host machine terminal:

cd session4

./enter_container.sh



# Inside the new Docker terminal:

source ~/ir_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard

```



### Session 5 - Connect the Mind of Your Robot

To be completed.



### Session 6 - Bring Intelligence to Your Robot

To be completed.



## Project

Not yet defined.
