cmake_minimum_required(VERSION 3.5)
project(pet_bot_test5)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install all necessary directories in one block
# This ensures config/pet_bot_controllers.yaml is copied correctly
install(
  DIRECTORY launch urdf config meshes textures worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python control script
install(PROGRAMS
  scripts/starting_gait.py
  scripts/forward_gait.py
  scripts/troting_hc.py
  scripts/backward_gait.py
  scripts/sideward_gait.py
  scripts/llm_starting_gait.py
  scripts/navigation.py
  scripts/gazebo_pose_reader.py
   scripts/teleop_keyboard.py
  scripts/rotate_gait.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
