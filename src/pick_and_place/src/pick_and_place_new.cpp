#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("pick_and_place", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.hpp>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.hpp>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.clearPathConstraints();
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.05;
  move_group.setPoseTarget(target_pose1);

  move_group.move();


  rclcpp::shutdown();
  return 0;
}

