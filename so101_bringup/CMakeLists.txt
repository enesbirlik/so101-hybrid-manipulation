cmake_minimum_required(VERSION 3.8)
project(so101_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/so101_ready.py scripts/so101_lift.py scripts/so101_teach_check.py
  scripts/so101_gripper_squeeze.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibration_consistency test/test_calibration_consistency.py)
endif()

ament_package()
