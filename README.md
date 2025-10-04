# Obstacle Avoidance Robot

A ROS-based autonomous robot system for Urban Search and Rescue operations with obstacle avoidance and victim detection capabilities.

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![C++](https://img.shields.io/badge/C++-11-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Overview

This project simulates a disaster response scenario where robots autonomously explore unknown environments, build maps, and locate victims using fiducial markers. The system is designed to assist first responders by providing accurate maps with victim locations in collapsed building scenarios.

## Features

- **Autonomous Navigation** - Self-directed exploration and pathfinding
- **Real-time Obstacle Avoidance** - Dynamic obstacle detection and avoidance
- **Multi-Robot Coordination** - Explorer and follower robot collaboration
- **SLAM-based Mapping** - Simultaneous localization and mapping
- **Victim Detection** - Fiducial marker-based victim identification
- **Gazebo Simulation** - Realistic physics-based environment
- **RViz Visualization** - Real-time monitoring and control

## Demo

Check out the demonstration videos in the `video/` folder:
- `obstacle.mp4` - Basic obstacle avoidance demonstration
- `obstacle_update.mp4` - Updated features and improvements

## Prerequisites

- **ROS Noetic** or Melodic
- **Ubuntu 20.04** (for Noetic) or 18.04 (for Melodic)
- **Gazebo 11**
- **RViz**
- **C++11** compiler

### Required ROS Packages

Install the necessary dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-tf2-ros
sudo apt-get install ros-noetic-fiducial-msgs
sudo apt-get install ros-noetic-gazebo-ros
sudo apt-get install ros-noetic-amcl
Note: Replace noetic with melodic if using ROS Melodic
Installation
1. Clone the Repository
bashcd ~/catkin_ws/src
git clone https://github.com/akhiljoshi7060/obstacle-avoidance-robot.git final_project_809y
2. Install Dependencies
bashcd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
3. Build the Project
bashcd ~/catkin_ws
catkin_make
source devel/setup.bash
Tip: Add source ~/catkin_ws/devel/setup.bash to your ~/.bashrc file for automatic sourcing
Usage
Step 1: Launch the Simulation Environment
Open a terminal and run:
bashroslaunch final_project_809y multiple_robots.launch
This command will:

Start Gazebo with the disaster environment
Launch RViz for visualization
Initialize the navigation stack
Load robot models and sensors

Step 2: Run the Main Control Node
In a new terminal:
bashsource ~/catkin_ws/devel/setup.bash
rosrun final_project_809y final_project_809y_node
This starts the autonomous exploration and obstacle avoidance system.
Step 3: Set Navigation Goals (Optional)

In RViz, click the "2D Nav Goal" button in the toolbar
Click and drag on the map to set a destination point for the robot
The robot will autonomously navigate to the goal while avoiding obstacles

Project Structure
final_project_809y/
├── config/              # Navigation and costmap configurations
├── launch/              # ROS launch files
│   └── multiple_robots.launch
├── maps/                # Pre-built occupancy grid maps
├── models/              # Gazebo robot and object models
├── rviz/                # RViz configuration files
├── scripts/             # Utility scripts
├── video/               # Demo videos
│   ├── obstacle.mp4
│   └── obstacle_update.mp4
├── worlds/              # Gazebo world files
├── include/             # C++ header files
│   └── final_project_809y/
│       ├── explorer_robot.h
│       └── follower_robot.h
├── src/                 # C++ source files
│   ├── main.cpp
│   ├── explorer_robot.cpp
│   └── follower_robot.cpp
├── CMakeLists.txt       # CMake build configuration
├── package.xml          # ROS package manifest
├── README.md            # This file
└── LICENSE              # MIT License
Technologies Used
TechnologyPurposeROS (Robot Operating System)Framework for robot software developmentC++11Core implementation languageGazebo3D physics-based simulation environmentRViz3D visualization tool for ROSNavigation StackPath planning and obstacle avoidancemove_baseAction-based navigation controllerAMCLAdaptive Monte Carlo LocalizationTF/TF2Coordinate frame transformationsactionlibAction-based asynchronous communication
ROS Architecture
Key Nodes:

final_project_809y_node - Main control and coordination
move_base - Navigation and path planning
amcl - Robot localization
map_server - Provides map data

Important Topics:

/cmd_vel - Velocity commands to robot
/scan - Laser scanner data
/odom - Odometry information
/map - Occupancy grid map
/fiducial_transforms - Victim marker positions

How It Works

Initialization: The system loads the environment and spawns robots
Exploration: The explorer robot autonomously navigates unexplored areas
Mapping: SLAM algorithms create a real-time map of the environment
Obstacle Detection: Laser scanners detect obstacles in real-time
Path Planning: The navigation stack computes collision-free paths
Victim Detection: Fiducial markers identify victim locations
Coordination: The follower robot tracks and assists the explorer

Troubleshooting
Build Errors
bashcd ~/catkin_ws
catkin_make clean
catkin_make
Missing Dependencies
bashrosdep update
rosdep install --from-paths src --ignore-src -r -y
Simulation Not Starting

Ensure Gazebo is properly installed
Check for port conflicts: killall gzserver gzclient
Verify ROS environment: echo $ROS_DISTRO

Robot Not Moving

Check that all nodes are running: rosnode list
Verify topics are publishing: rostopic list
Ensure navigation goals are set correctly in RViz

License
This project is licensed under the MIT License - see the LICENSE file for details.
Author
Akhil Joshi

GitHub: @akhiljoshi7060

Acknowledgments

ROS Community for the excellent framework
Gazebo Team for the simulation environment
Urban Search and Rescue robotics research community

Contact
For questions, issues, or contributions:

Open an issue on GitHub Issues
Pull requests are welcome!

Future Improvements

 Real robot hardware integration
 Advanced AI-based path planning
 Multiple victim detection algorithms
 Cloud-based mission control
 Multi-floor navigation support


Star ⭐ this repository if you find it helpful!

License
markdown## License
MIT License

Purpose: Legal protection and usage rights
MIT: Permissive license (allows others to use freely)
Important: Required for open-source projects
