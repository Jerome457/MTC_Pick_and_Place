from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_urdf").to_moveit_configs()

    # MTC Demo node
    pick_place_demo = Node(
        package="arm_urdf",
        executable="pick_and_place",
        output="screen",
        parameters=[moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])