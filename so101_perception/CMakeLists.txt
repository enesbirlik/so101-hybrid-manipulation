cmake_minimum_required(VERSION 3.8)
project(so101_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/wrist_camera_node.py scripts/cube_detector_node.py scripts/wood_cube_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)
# imported by the detector nodes from their own directory
install(FILES scripts/cube_detection.py scripts/cube_tracking.py scripts/wood_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cube_detection test/test_cube_detection.py TIMEOUT 180)
  ament_add_pytest_test(test_cube_tracking test/test_cube_tracking.py)
  ament_add_pytest_test(test_wood_detection test/test_wood_detection.py TIMEOUT 180)
endif()

ament_package()
