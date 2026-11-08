cmake_minimum_required(VERSION 3.16)
project(so101_dashboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

# raylib, Dear ImGui, rlImGui and ImPlot are not packaged for ROS: fetched at pinned commits on the first build.
include(FetchContent)
set(FETCHCONTENT_QUIET ON)
FetchContent_Declare(raylib GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG c1ab645ca298a2801097931d1079b10ff7eb9df8 GIT_SHALLOW FALSE)   # 5.5
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.92.9-docking GIT_SHALLOW TRUE)
FetchContent_Declare(rlimgui GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
  GIT_TAG 1550009359ad975927f7f0e4a3f47e3f27123ea9)
FetchContent_Declare(implot GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG 819f200)   # v1.1

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)
FetchContent_Populate(imgui)
FetchContent_Populate(rlimgui)
FetchContent_Populate(implot)

add_library(dashboard_gui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${rlimgui_SOURCE_DIR}/rlImGui.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp)
target_include_directories(dashboard_gui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${rlimgui_SOURCE_DIR} ${implot_SOURCE_DIR})
target_link_libraries(dashboard_gui PUBLIC raylib)
target_compile_options(dashboard_gui PRIVATE -w)
# 32-bit indices: a 60 s plot of six joints is more than ImGui's default 65536 vertices per window
target_compile_definitions(dashboard_gui PUBLIC "ImDrawIdx=unsigned int")

add_executable(dashboard
  src/main.cpp
  src/ros_link.cpp
  src/theme.cpp
  src/panels.cpp
  src/robot_view.cpp
  src/camera_view.cpp
  src/tasks.cpp
  src/layout.cpp)
target_link_libraries(dashboard dashboard_gui ${OpenCV_LIBS} tinyxml2::tinyxml2)
target_compile_options(dashboard PRIVATE -Wall -Wextra)   # ours only, not the fetched libraries
target_compile_definitions(dashboard PRIVATE NO_FONT_AWESOME)
target_compile_definitions(dashboard_gui PRIVATE NO_FONT_AWESOME)
ament_target_dependencies(dashboard rclcpp ament_index_cpp sensor_msgs std_msgs tf2_ros urdf visualization_msgs control_msgs moveit_msgs rclcpp_action controller_manager_msgs)

install(TARGETS dashboard DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY fonts launch DESTINATION share/${PROJECT_NAME})

ament_package()
