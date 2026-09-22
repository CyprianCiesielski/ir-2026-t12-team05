# Intelligent Robotics - Johny
Intelligent Robotics 2026 (UPC-EEBE) – T12 Team 05. Practice sessions and final project.

## Team Information
- **Team members:** Cyprian Ciesielski, Laila Fahchouch
- **Practice group:** T12
- **Team number:** Team05
- **Robot name:** Johny

## Repository Structure
```
ir-2026-t12-team05
├── README.md
├── session1
│   ├── robot_startup.sh
│   └── team_info.txt
└── session2
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
To be completed.

### Session 4 - Bring Your Robot to Life
To be completed.

### Session 5 - Connect the Mind of Your Robot
To be completed.

### Session 6 - Bring Intelligence to Your Robot
To be completed.

## Project
Not yet defined.
