### Execution
Stage 1 - Rebuild
```bash
cd ~/Zhang_25252980_EE650_ws
colcon build --packages-select smart_energy_management_robot
source install/setup.bash
```
Stage 2 - Test Order
1. Gazebo
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
2. Nav2 with RViz
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/Maps/house_explored.yaml \
  use_sim_time:=True
```
3. PlanSys2
```bash
ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
  model_file:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/pddl/domain.pddl
```
What you should see (All four components must show Activated before moving on.)
```bash
[plansys2_node-1] [domain_expert]: [domain_expert] Activated
[plansys2_node-1] [problem_expert]: [problem_expert] Activated
[plansys2_node-1] [planner]: [planner] Activated
[plansys2_node-1] [executor]: [executor] Activated
```
4. Action Executor
```bash
ros2 run smart_energy_management_robot visit_action_executor.py
```
What you should see
```bash
[visit_action_executor]: VisitActionExecutor ready
# Then it waits silently for actions from PlanSys2.
```

5. Problem Generator (trigger the plan)
```bash
ros2 run smart_energy_management_robot problem_generator.py
```


### Preparation:
Stage 1 — Test Gazebo House World
Open a new terminal:
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
Verify the Gazebo window opens with the house environment and the robot spawns. Share a screenshot or confirm it works before moving to Stage 2.

Optional — Test PlanSys2 With Our Domain
Open another new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
  model_file:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/pddl/domain.pddl
```
Then in another terminal verify PlanSys2 loaded the domain correctly:
```bash
source /opt/ros/humble/setup.bash
ros2 service call /domain_expert/get_domain \
  plansys2_msgs/srv/GetDomain {}
```
You should see your domain PDDL returned in the response.

Stage 2 - Set initial position in RViz
```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Click 2D Pose Estimate button on RViz, then click on the map and drag in the rough direction it faces.

Stage 3 — Nav2
Open another terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/Maps/house_explored.yaml \
  use_sim_time:=True
```

Stage 4 - Get the location
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /odom --once
```

Stage 5 - teleop 
```bash
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

