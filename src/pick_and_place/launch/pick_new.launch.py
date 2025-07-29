from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_urdf").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="pick_and_place",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    spawn_cylinder = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(get_package_share_directory("arm_urdf"), "urdf", "cylinder.urdf"),
            "-entity", "cylinder",
            "-x", "0.7", "-y", "0.0", "-z", "0.05"
        ],
        output="screen"
    )

    spawn_monitor= Node(
            package='pick_and_place',  # 🔁 Replace with your package name
            executable='gripper_monitor',
            output='screen'
        )

    return LaunchDescription([pick_place_demo, spawn_cylinder,spawn_monitor])