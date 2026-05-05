cmake_minimum_required(VERSION 3.8)
project(smart_energy_management_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  behavior_trees_xml config Maps launch pddl src
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/problem_generator.py
  src/visit_action_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
