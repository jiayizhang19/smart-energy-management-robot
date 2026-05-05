Stage 1 — Test Gazebo House World
Open a new terminal:
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
Verify the Gazebo window opens with the house environment and the robot spawns. Share a screenshot or confirm it works before moving to Stage 2.

Stage 2 — Test PlanSys2 With Our Domain
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

Stage 3 — Test Nav2
Open another terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Zhang_25252980_EE650_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/Zhang_25252980_EE650_ws/src/smart_energy_management_robot/Maps/house_explored.yaml \
  use_sim_time:=True
```
RViz should open showing the map. Set the 2D Pose Estimate in RViz to align the robot with the map.