cmake_minimum_required(VERSION 3.8)
project(so101_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/wrist_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
