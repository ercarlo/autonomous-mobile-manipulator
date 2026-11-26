# Installation and Usage Guide

This document describes the development environment and the main commands used to launch the Autonomous Mobile Manipulator system.

## 1. System Requirements

The project was developed and tested with the following software stack:

| Component | Technology |
|---|---|
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Navigation | Nav2 |
| Manipulation | MoveIt 2 |
| Robot control | ros2_control |
| Object detection | Ultralytics YOLO11 |
| Image processing | OpenCV |
| Programming | Python 3 |
| Coordinate transforms | TF2 |

A computer with suitable graphics acceleration is recommended for running Gazebo, MoveIt 2, Nav2 and RGB-D perception simultaneously.

## 2. Workspace Setup

Once the project source packages and required model assets are available locally, create a ROS 2 workspace:

```bash
mkdir -p ~/autonomous_mobile_manipulator/src
cd ~/autonomous_mobile_manipulator
```

Place the project's ROS 2 packages in the `src/` directory.

Make sure that ROS 2 Jazzy and all required project dependencies have been installed before building.

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

**Note:** The full dependency manifest and source code are being prepared for publication. The repository should not be considered independently reproducible until those files and third-party asset instructions are included.

## 3. Launch the Simulation

Start Gazebo Harmonic with the mobile manipulator and industrial manipulation scene:

```bash
ros2 launch amm_gazebo manipulation_scene.launch.py
```

The simulation includes:

- MiR100 autonomous mobile platform.
- UR5e robotic manipulator.
- Robotiq 2F-85 parallel gripper.
- Wrist-mounted RGB-D camera.
- Industrial workbench.
- YCB objects used for manipulation experiments.

## 4. Launch MoveIt 2

In a new terminal, source the workspace and start the manipulation planning server:

```bash
LC_NUMERIC=C ros2 launch amm_moveit_config \
  move_group.launch.py \
  allow_trajectory_execution:=true
```

An optional RViz configuration can be started for visualizing motion planning and robot state:

```bash
LC_NUMERIC=C QT_QPA_PLATFORM=xcb \
ros2 launch amm_moveit_config \
  moveit_rviz_gazebo.launch.py
```

## 5. Execute an Autonomous Mission

The mission coordinator accepts a target object and a destination station.

For example:

```bash
ros2 run amm_manipulation task_coordinator \
  --object apple \
  --destination mesa1 \
  --execute true \
  --ros-args -p use_sim_time:=true
```

The coordinator is designed to orchestrate the following workflow:

1. Initialize localization and record the starting position.
2. Activate RGB-D perception and object detection.
3. Estimate the target object's three-dimensional position.
4. Execute collision-aware robotic grasping.
5. Transition to autonomous navigation.
6. Travel to the selected destination station.
7. Execute the object placement procedure.
8. Return to the starting position.

To verify mission configuration without commanding motion, use:

```bash
ros2 run amm_manipulation task_coordinator \
  --object apple \
  --destination mesa1 \
  --execute false \
  --ros-args -p use_sim_time:=true
```

## 6. Navigation Subsystem

Localization and navigation can also be started individually for testing:

```bash
ros2 launch amm_slam localization.launch.py
```

In a separate terminal:

```bash
ros2 launch amm_navigation navigation.launch.py
```

The navigation stack incorporates AMCL localization, NavFn global planning, Regulated Pure Pursuit control and behavior-tree-based navigation.

## 7. Troubleshooting

### Motion planning fails

Verify that the MoveIt 2 planning server is active, the robot's joint states are being published and the destination pose is reachable without collisions.

### Missing workbench collision geometry

The grasping safety checks require the workbench to be represented in the MoveIt Planning Scene. Confirm that `amm_workbench_top` is present before attempting a Cartesian descent.

### Navigation goal rejected

Check the lifecycle state of the navigation servers:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

The required servers should report `active [3]`.

### Perception processes already running

Do not start duplicate perception nodes when using the mission coordinator. Check for existing YOLO, 3D object-position and detected-object scene processes before launching a new coordinated mission.

## 8. Demonstrations and Documentation

The repository README provides animated demonstrations of the autonomous picking and delivery workflow.

For additional technical details, consult the [Engineering Report](technical_report.pdf).

---

**Environment:** ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic

**Project:** Autonomous Mobile Manipulator
