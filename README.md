# 🤖 Obstacle Avoidance Robot

**A ROS-based autonomous robot system for Urban Search and Rescue operations with obstacle avoidance and victim detection capabilities.**

---

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![C++](https://img.shields.io/badge/C++-11-blue) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

## 📋 Overview

This project simulates disaster response scenarios where robots autonomously explore unknown environments (like collapsed buildings), build maps, and locate victims using fiducial markers. The system helps first responders by providing accurate maps with marked victim locations.

**Key Components:**

* **Explorer Robot:** Primary navigation and mapping
* **Follower Robot:** Secondary support and victim detection
* **ArUco Markers:** Victim location identification

---

## ✨ Features

* ✅ **Autonomous Navigation:** Self-directed exploration and pathfinding
* ✅ **Real-time Obstacle Avoidance:** Dynamic obstacle detection
* ✅ **Multi-Robot Coordination:** Explorer and follower collaboration
* ✅ **SLAM-based Mapping:** Simultaneous localization and mapping
* ✅ **Victim Detection:** ArUco fiducial marker identification
* ✅ **Gazebo Simulation:** Realistic physics environment
* ✅ **RViz Visualization:** Real-time monitoring

---

## 🎥 Demo Videos

Check out demonstration videos in the `video/` folder:

* **obstacle** - Robot navigation and obstacle avoidance demo
* **obstacle_1** - Additional demonstration footage

---

## 🛠️ Prerequisites

**Operating System:**

* Ubuntu 20.04 LTS (for ROS Noetic) - Recommended
* Ubuntu 18.04 LTS (for ROS Melodic)

**Required Software:**

* ROS Noetic or Melodic
* Gazebo 11
* RViz
* C++11 compiler

**Install ROS Packages:**

```bash
# Update system
sudo apt-get update

# Install Navigation Stack
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-tf2-ros
sudo apt-get install ros-noetic-fiducial-msgs
sudo apt-get install ros-noetic-gazebo-ros
sudo apt-get install ros-noetic-amcl
```

> **Note:** Replace `noetic` with `melodic` if using ROS Melodic.

---

## 📦 Installation

**Step 1: Create Catkin Workspace**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Step 2: Clone Repository**

```bash
cd ~/catkin_ws/src
git clone https://github.com/akhiljoshi7060/obstacle-avoidance-robot.git final_project_809y
```

**Step 3: Install Dependencies**

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Step 4: Build Project**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Usage

**Step 1: Launch Simulation**

```bash
roslaunch final_project_809y multiple_robots.launch
```

This will start:

* Gazebo simulation environment
* RViz visualization
* Navigation stack
* Both robots

**Step 2: Run Main Node**

```bash
source ~/catkin_ws/devel/setup.bash
rosrun final_project_809y final_project_809y_node
```

**Step 3: Set Navigation Goals (Optional)**

* In RViz, click **"2D Nav Goal"**
* Click and drag on the map to set destination
* Robot will navigate autonomously

---

## 📁 Project Structure

```
final_project_809y/
├── config/                  # Configuration files
│   ├── include/            # Header files
│   │   ├── explorer_robot.h
│   │   └── follower_robot.h
│   └── param/              # Parameter files
│       ├── aruco_lookup.yaml
│       ├── costmap_common_params_explorer.yaml
│       ├── costmap_common_params_follower.yaml
│       └── dwa_local_planner_params.yaml
│
├── launch/                  # Launch files
│   ├── multiple_robots.launch
│   ├── explorer_amcl.launch
│   ├── follower_amcl.launch
│   └── mapping/
│
├── maps/                    # Map files
│   ├── final_world.pgm
│   └── final_world.yaml
│
├── models/                  # Gazebo models
│   ├── aruco_marker_0/
│   ├── aruco_marker_1/
│   ├── aruco_marker_2/
│   └── aruco_marker_3/
│
├── rviz/                    # RViz configs
│   ├── bringup.rviz
│   └── navigation.rviz
│
├── scripts/                 # Scripts and source
│   ├── install.bash
│   └── src/
│       ├── main.cpp
│       ├── explorer_robot.cpp
│       └── follower_robot.cpp
│
├── video/                   # Demo videos
│   ├── obstacle
│   └── obstacle_1
│
├── worlds/                  # Gazebo worlds
│   └── final_world.world
│
├── CMakeLists.txt
├── package.xml
├── README.md
└── LICENSE
```

---

## 🔧 Technologies Used

| Technology           | Purpose                      |
| -------------------- | ---------------------------- |
| **ROS**              | Core robotics framework      |
| **C++11**            | Primary programming language |
| **Gazebo**           | 3D physics simulation        |
| **RViz**             | 3D visualization             |
| **Navigation Stack** | Path planning and navigation |
| **move_base**        | Navigation controller        |
| **AMCL**             | Robot localization           |
| **TF/TF2**           | Coordinate transformations   |

---

## 🎯 How It Works

1. **Initialization:** System loads environment and spawns robots
2. **Exploration:** Explorer robot autonomously navigates
3. **Mapping:** SLAM creates real-time maps
4. **Obstacle Detection:** Laser scanners detect obstacles
5. **Path Planning:** Navigation stack computes safe paths
6. **Victim Detection:** ArUco markers identify victims
7. **Coordination:** Follower robot assists explorer

---

## 🐛 Troubleshooting

**Build Errors**

```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

**Missing Dependencies**

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Gazebo Not Starting**

```bash
killall gzserver gzclient
gazebo --version
```

**Robot Not Moving**

```bash
rosnode list
rostopic echo /cmd_vel
```
This project is licensed under the **MIT License**.

```
MIT License

Copyright (c) 2025 Akhil Joshi
