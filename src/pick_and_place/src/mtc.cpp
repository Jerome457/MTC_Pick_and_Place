#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
int e=0;
int num_objects = 3;
double start_x = 0.3;
// double end_x = 0.7;
double spacing = 0.1;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(int e);

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(int i);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
{
  // ----- Add Multiple Cylinders -----
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::vector<moveit_msgs::msg::ObjectColor> colors;


  for (int i = 0; i < num_objects; ++i) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "object_" + std::to_string(i + 1);
    obj.header.frame_id = "world";

    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    obj.primitives[0].dimensions = {0.1, 0.02};  // height, radius

    geometry_msgs::msg::Pose pose;
    pose.position.x = start_x + i * spacing;
    pose.position.y = 0.0;
    pose.position.z = 0.05;  // above ground
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    obj.operation = obj.ADD;
    objects.push_back(obj);

    // Assign a color (e.g., alternating)
    moveit_msgs::msg::ObjectColor color;
    color.id = obj.id;
    color.color.r = (i % 3 == 0) ? 1.0 : 0.0;  // red every 3rd
    color.color.g = (i % 3 == 1) ? 1.0 : 0.0;  // green
    color.color.b = (i % 3 == 2) ? 1.0 : 0.0;  // blue
    color.color.a = 1.0;
    colors.push_back(color);
  }

  // ----- Add Ground Plane -----
  moveit_msgs::msg::CollisionObject ground;
  ground.id = "ground_plane";
  ground.header.frame_id = "base_link";  // or your planning frame

  shape_msgs::msg::SolidPrimitive ground_shape;
  ground_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  ground_shape.dimensions = {4.0, 4.0, 0.01};

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.position.z = -0.05;
  ground_pose.orientation.w = 1.0;

  ground.primitives.push_back(ground_shape);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;

  moveit_msgs::msg::ObjectColor ground_color;
  ground_color.id = "ground_plane";
  ground_color.color.r = 0.0;
  ground_color.color.g = 0.0;
  ground_color.color.b = 1.0;
  ground_color.color.a = 0.0;

  // ----- Publish Planning Scene -----
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;

  // Add all objects
  for (auto& obj : objects) scene_msg.world.collision_objects.push_back(obj);
  scene_msg.world.collision_objects.push_back(ground);

  // Add all colors
  for (auto& col : colors) scene_msg.object_colors.push_back(col);
  scene_msg.object_colors.push_back(ground_color);

  // Publish multiple times to ensure reception
  auto planning_scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  rclcpp::Rate rate(10);
  for (int i = 0; i < 5; ++i) {
    planning_scene_pub->publish(scene_msg);
    rate.sleep();
  }
}
}

void MTCTaskNode::doTask(int a)
{
  task_ = createTask(a);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    --e;
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    --e;
    return;
  }
// moveit_task_constructor_msgs::msg::Solution empty_solution;
// solution_pub_->publish(empty_solution);


  return;
}

mtc::Task MTCTaskNode::createTask(int i)
{
  mtc::Task task;
  task.stages()->setName("demo task" + std::to_string(i + 1));
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "grasp_frame";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("RRTConnect");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
stage_move_to_pick->setTimeout(5.0);
stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
task.add(std::move(stage_move_to_pick));

mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

      {
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.15);
      
        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.x = -0.05;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      {
        // Sample grasp pose
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object_" + std::to_string(i + 1));
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI/10, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation() = Eigen::Vector3d(-0.025, 0.0, 0.0);
        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }

      {
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (soft_fingers,object)");
        stage->allowCollisions("object_" + std::to_string(i + 1), "Soft_finger_1", true);
        stage->allowCollisions("object_" + std::to_string(i + 1), "Soft_finger_2", true);
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("close");
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object" + std::to_string(i + 1));
        stage->attachObject("object_" + std::to_string(i + 1), hand_frame);
        attach_object_stage = stage.get();
        grasp->insert(std::move(stage));
      }


      {
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.3);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "lift_object");
      
        // Set upward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      task.add(std::move(grasp));
    }
    
    {
      auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                    { hand_group_name, sampling_planner } });
      stage_move_to_place->setTimeout(5.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_place));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

      {
        // Sample place pose
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject("object_" + std::to_string(i + 1));
      
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "base_footprint";
        target_pose_msg.pose.position.y = 0.2-0.1*i;
        target_pose_msg.pose.position.x = -0.5;
        target_pose_msg.pose.position.z=0.05;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
      
        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame("object_" + std::to_string(i + 1));
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        place->insert(std::move(stage));
      }
      {
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions("object_" + std::to_string(i + 1), "Soft_finger_1", false);
        stage->allowCollisions("object_" + std::to_string(i + 1), "Soft_finger_2", false);
        place->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject("object_" + std::to_string(i + 1), hand_frame);
        place->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.3);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "retreat");
      
        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.x = 0.025;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      task.add(std::move(place));
    }
    if(i==num_objects-1)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("ready");
      task.add(std::move(stage));
    }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  for(e=0;e<num_objects;e++)
    mtc_task_node->doTask(e);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}