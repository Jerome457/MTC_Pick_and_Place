cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Add these dependencies
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)  # Recent addition
# find_package(robo_moveit_config REQUIRED)
find_package(sensor_msgs REQUIRED) # Very Recent Addition
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(linkattacher_msgs)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the executable and link dependencies
add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place 
    moveit_ros_planning_interface 
    rclcpp 
    geometry_msgs 
    tf2_geometry_msgs 
    std_msgs
    shape_msgs
    sensor_msgs # Very Recent Addition
)

add_executable(gripper_monitor src/gazebo_link_attacher.cpp)
ament_target_dependencies(gripper_monitor
  rclcpp
sensor_msgs 
std_srvs
linkattacher_msgs
)


add_executable(mtc_tutorial src/mtc.cpp)

ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp linkattacher_msgs)
target_include_directories(mtc_tutorial PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mtc_tutorial 
DESTINATION lib/${PROJECT_NAME})


install(TARGETS gripper_monitor
  DESTINATION lib/${PROJECT_NAME})


# Install the target
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(TARGETS pick_and_place DESTINATION lib/${PROJECT_NAME})

ament_package()
