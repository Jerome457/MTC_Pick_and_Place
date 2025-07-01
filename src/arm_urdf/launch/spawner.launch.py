from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
import os
import xacro

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Define robot description files
    robot_description_file = os.path.join(
        get_package_share_directory("arm_urdf"),
        "urdf",
        "arm_urdf.xacro"
    )

    robot_description_config = xacro.process_file(robot_description_file)
    robot_urdf = robot_description_config.toxml()

    joint_controllers_file = os.path.join(
        get_package_share_directory("arm_urdf_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    # Robot description parameter
    # robot_description = Command(["xacro ", robot_urdf])  # Fixed extra space issue

    # MoveIt2 Configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_urdf")
        .robot_description(file_path="config/arm_urdf.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_urdf.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "debug": "false",
            "gui": "true",
            "paused": "true",
        }.items()
    )

    # RViz Config
    rviz_config_path = os.path.join(
        get_package_share_directory("arm_urdf_moveit_config"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ]
    )


    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, joint_controllers_file],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Spawn the robot in Gazebo
    spawn_the_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "arm_urdf",
            "-topic", "/robot_description",
        ],
        output="screen"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description]
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Move Group Node
    config_dict = moveit_config.to_dict()
    config_dict.update({"use_sim_time": True})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # Delayed Start Handlers
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[hand_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    # Launch Description
    ld = LaunchDescription()

    # Add launch actions
    ld.add_action(gazebo)
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(spawn_the_robot)
    ld.add_action(delay_rviz_node)
    ld.add_action(move_group_node)


    return ld  # Correct return statement
