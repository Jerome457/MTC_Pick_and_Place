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

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                latest_joint_values_.clear();
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    latest_joint_values_[msg->name[i]] = msg->position[i];
                }
            });
    }

    /// 🚀 **Function to add the cylinder**
    void add_cylinder()
    {
        auto scene = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.id = "cylinder";
        cylinder.header.frame_id = "world";  // Match your frame

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


    void print_latest_joint_values()
    {
        if (latest_joint_values_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Joint state not yet received.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Latest Joint Angles from /joint_states:");
        for (const auto &pair : latest_joint_values_)
        {
            double radians = pair.second;
            double degrees = radians * (180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  %s: %f rad (%f deg)", pair.first.c_str(), radians, degrees);
        }
    }


    void stm_comm()
    {
        if (latest_joint_values_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No joint values available to send to STM.");
            return;
        }

        std::ostringstream json_stream;
        json_stream << "{";

        size_t count = 0;
        for (const auto &pair : latest_joint_values_)
        {
            double degrees = pair.second * (180.0 / M_PI);  // Convert to degrees
            json_stream << "\"" << pair.first << "\": " << degrees;
            if (++count < latest_joint_values_.size())
                json_stream << ", ";
        }

        json_stream << "}";

        std::string json_data = json_stream.str();

        // Replace with your actual script path
        std::string script_path = "/home/soham/ws_moveit2_Tren/src/pick_and_place/scripts/send_angles.py";
        std::string command = "python3 " + script_path + " '" + json_data + "'";

        int result = system(command.c_str());

        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "STM acknowledged with ACK.");
        }
        else if (result == 2)
        {
            RCLCPP_WARN(this->get_logger(), "STM responded but did not ACK.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to communicate with STM. Code: %d", result);
        }
    }



    /// 🚀 **Pick-and-place sequence**
    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Trying to move arm");
        // Move arm to home position
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "arm");

        add_cylinder();

        // move_group->setNamedTarget("home");
        // move_group->move();


        // 🛠️ Define pick and place poses
        geometry_msgs::msg::PoseStamped pick_pose;
        pick_pose.header.frame_id = "base_footprint";
        pick_pose.pose.position.x = 0.5;
        pick_pose.pose.position.y = 0.0;
        pick_pose.pose.position.z = 0.05;  // Just above the cylinder
        // pick_pose.pose.orientation.w = 1.0;
        

        geometry_msgs::msg::PoseStamped place_pose = pick_pose;
        place_pose.pose.position.x = -0.5;

        // 🛠️ Pick motion
        move_group->setPoseTarget(pick_pose);
        move_group->move();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // print_latest_joint_values();
        // stm_comm();

        // 🛠️ Gripper motion
        auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "hand");

        gripper_group->setNamedTarget("close");
        gripper_group->move();

        // 🛠️ Place motion
        move_group->setPoseTarget(place_pose);
        move_group->move();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // print_latest_joint_values();
        // stm_comm();


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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class PickAndPlace : public rclcpp::Node
// {
// public:
//     PickAndPlace() : Node("pick_and_place")
//     {
//         RCLCPP_INFO(this->get_logger(), "PickAndPlace node initialized.");
//     }

//     void run()
//     {
//         // Use rclcpp::Node's shared_from_this() explicitly
//         auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             this->shared_from_this(), "arm");

//         move_group->setNamedTarget("home");
//         move_group->move();

//         geometry_msgs::msg::PoseStamped pick_pose;
//         pick_pose.header.frame_id = "base_footprint";  //was initally world
//         pick_pose.pose.position.x = 0.5;
//         pick_pose.pose.position.y = 0.0;
//         pick_pose.pose.position.z = 0.2;
//         pick_pose.pose.orientation.w = 1.0;

//         geometry_msgs::msg::PoseStamped place_pose = pick_pose;
//         place_pose.pose.position.x = -0.5;

//         move_group->setPoseTarget(pick_pose);
//         move_group->move();

//         auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             this->shared_from_this(), "gripper");

//         gripper_group->setNamedTarget("closed");
//         gripper_group->move();

//         move_group->setPoseTarget(place_pose);
//         move_group->move();

//         gripper_group->setNamedTarget("open");
//         gripper_group->move();

//         RCLCPP_INFO(this->get_logger(), "Pick-and-place completed.");
//     }
// };


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PickAndPlace>(); // Now it's a shared_ptr
//     node->run(); // Execute the pick-and-place sequence
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
