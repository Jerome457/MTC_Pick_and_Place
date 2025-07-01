from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm_urdf", package_name="arm_urdf_moveit_config")
        .robot_description(file_path="config/arm_urdf.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_urdf.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="pick_and_place",
        package="pick_and_place",
        executable="pick_and_place",
        output="screen",
        parameters=[moveit_config.to_dict() 
        ],
    )

    return LaunchDescription([move_group_demo])