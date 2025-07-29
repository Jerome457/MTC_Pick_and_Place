import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("arm_urdf")
        .robot_description(file_path="config/arm_urdf.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_urdf.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # Add sim time globally
    sim_time = {"use_sim_time": True}

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            sim_time,
        ],
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("arm_urdf_moveit_config") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            sim_time,
        ],
    )

    # # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    # )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            sim_time,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_urdf_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path,sim_time],
        output="both",
    )
        # Gazebo Launch
    world_file = os.path.join(
        get_package_share_directory("arm_urdf"),
        "config",
        "arm.world"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "debug": "false",
            "gui": "true",
            "paused": "true",
            "world": world_file,
        }.items()
    )
    spawn_the_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "arm_urdf",
            "-topic", "/robot_description",
        ],
        output="screen"
    )

    

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} --controller-manager /controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            gazebo,                     # 1. Launch Gazebo first
            spawn_the_robot,           # 2. Then spawn robot into Gazebo
            ros2_control_node,         # 3. Then launch ros2_control_node (after robot is in Gazebo)
            robot_state_publisher,
            run_move_group_node,
            rviz_node,
        ]
        + load_controllers
    )