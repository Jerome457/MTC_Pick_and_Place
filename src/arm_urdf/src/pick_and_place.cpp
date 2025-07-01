#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath> //To convert radians to degrees
#include <unordered_map>  //Added this for convenience of getting value, no changes needed in CMake and pkg.xml
#include <sensor_msgs/msg/joint_state.hpp> //added this to subscribe to joint state
class PickAndPlace : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;   //Added this as a part of the new method os subscribing to joint state
    std::unordered_map<std::string, double> latest_joint_values_;
public:
    PickAndPlace() : Node("pick_and_place")
    {
        //this->declare_parameter("use_sim_time", true); //Added this to simulate time //Caused error
        RCLCPP_INFO(this->get_logger(), "PickAndPlace node initialized.");
        //add_cylinder();  // Add cylinder on initialization

    }

    /// 🚀 **Function to add the cylinder**
    void add_cylinder()
    {
        auto scene = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.id = "cylinder";
        cylinder.header.frame_id = "base_link";  // Match your frame

        // Define cylinder dimensions
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = 0.1;   // Height: 10 cm
        primitive.dimensions[1] = 0.03;  // Radius: 3 cm

        // Define cylinder pose
        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.position.x = 0.5;
        cylinder_pose.position.y = 0.0;
        cylinder_pose.position.z = 0.05;  // Half of the height
        cylinder_pose.orientation.w = 1.0;

        cylinder.primitives.push_back(primitive);
        cylinder.primitive_poses.push_back(cylinder_pose);
        cylinder.operation = moveit_msgs::msg::CollisionObject::ADD;

        scene->applyCollisionObject(cylinder);
        RCLCPP_INFO(this->get_logger(), "Cylinder added to the scene.");
    }


    /// 🚀 **Pick-and-place sequence**
    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Trying to move arm");
        // Move arm to home position
        // this->add_cylinder();
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "arm");

        // 🛠️ Gripper motion
        auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "hand");

        move_group->setNamedTarget("ready");
        move_group->move();


        gripper_group->setNamedTarget("open");
        gripper_group->move();


        // 🛠️ Define pick and place poses
        geometry_msgs::msg::PoseStamped pick_pose;
        pick_pose.header.frame_id="link1";
        pick_pose.pose.position.x = 1.00000;
        pick_pose.pose.position.y = 1.00000;
        pick_pose.pose.position.z = 1.00000;  // Just above the cylinder
        pick_pose.pose.orientation.w = 1.000000;
        

        geometry_msgs::msg::PoseStamped place_pose = pick_pose;
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        place_pose.pose.position.x = -0.5;

        // 🛠️ Pick motion
        if (!move_group->setPoseTarget(pick_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pose target.");
            return;
        }    

        move_group->setPlanningTime(45.0);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "Planning failed.");
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning to pose: x=%.3f y=%.3f z=%.3f",
        pick_pose.pose.position.x,
        pick_pose.pose.position.y,
        pick_pose.pose.position.z);
    

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        gripper_group->setNamedTarget("close");
        gripper_group->move();

        // 🛠️ Place motion
        move_group->setPoseTarget(place_pose);
        move_group->move();
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        gripper_group->setNamedTarget("open");
        gripper_group->move();

        RCLCPP_INFO(this->get_logger(), "Pick-and-place completed.");
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlace>();  // Now it's a shared_ptr

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Start spinning in a separate thread
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // Give it a little time to receive joint state
    rclcpp::sleep_for(std::chrono::seconds(1));

    node->run();                                   // Execute the pick-and-place sequence
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
