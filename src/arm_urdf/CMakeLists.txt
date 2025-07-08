cmake_minimum_required(VERSION 3.8)
project(arm_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(shape_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)
# Gazebo + URDF (if needed elsewhere)
# find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)

# add_executable(pick_and_place src/pick_and_place.cpp)

# ament_target_dependencies(pick_and_place 
#   rclcpp 
#   geometry_msgs 
#   tf2_geometry_msgs 
#   std_msgs
#   shape_msgs
#   sensor_msgs
#   moveit_ros_planning_interface
#   moveit_task_constructor_core
# )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY meshes urdf config DESTINATION share/${PROJECT_NAME}/)
# install(TARGETS pick_and_place DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
