# Smart Energy Management Robot

## Declaration
By submitting this assignment, I declare that this is all my own work and does not
include the work of any other person (from my class, from the internet, or elsewhere) or
GenAI output (except where specifically noted below)

## GenAI Usage
1. README.md content and structure.  
Prompt: "Write a README for this project, including what it is, how it works and how to run it."
2. visit_action_executor.py code structure and comments.  
Prompt: "Write a Python ROS 2 action executor node that subscribes to a topic for visiting waypoints, and commands Nav2 to navigate to those waypoints. Include comments explaining the code structure and functionality."

## Overview
This project simulates an energy-aware service robot that patrols a home, prioritizes rooms with
critical or high energy usage, and reduces waste by managing lighting when rooms are unoccupied.
It starts from the entrance spawn point and completes a full set of room visits.

Technically, it is a ROS 2 navigation and planning workflow for a TurtleBot3 in a house
environment. PlanSys2 generates a plan from PDDL domain/problem files and commands Nav2 to visit
waypoints. At each room, the action sequence is `CheckOccupancy`, `ManageLight`, and
`ResolveEnergy`, and the initial pose is always set to the entrance spawn position.

## How It Works
High-level flow:
1. Gazebo provides the TurtleBot3 house environment.
2. Nav2 handles localization and navigation on the prebuilt map.
3. PlanSys2 loads the PDDL domain and problem and produces a plan.
4. Action executor nodes carry out plan actions (e.g., visit waypoints).
5. A problem generator node triggers planning and execution.

Key components:
- PDDL: [src/smart_energy_management_robot/pddl/](src/smart_energy_management_robot/pddl/)
- Action executor: [src/smart_energy_management_robot/src/visit_action_executor.py](src/smart_energy_management_robot/src/visit_action_executor.py)
- Problem generator: [src/smart_energy_management_robot/src/problem_generator.py](src/smart_energy_management_robot/src/problem_generator.py)
- Launch file: [src/smart_energy_management_robot/launch/energy_management.launch.py](src/smart_energy_management_robot/launch/energy_management.launch.py)

## Prerequisites
- ROS 2 Humble
- TurtleBot3 Gazebo and Nav2 packages
- PlanSys2 packages

## Build
```bash
cd ~/Zhang_25252980_EE650_ws
colcon build --packages-select smart_energy_management_robot
source install/setup.bash
```

## Run (Simulation)
Open separate terminals for each stage.

1) Gazebo (house world)
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

2) Nav2 with RViz
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
	map:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/Maps/house_explored.yaml \
	use_sim_time:=True
```

3) PlanSys2 + Action Executor + Problem Generator
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
ros2 launch smart_energy_management_robot energy_management.launch.py
```




