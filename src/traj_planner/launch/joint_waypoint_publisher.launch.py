from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='traj_planner',
            executable='joint_waypoint_publisher',
            name='joint_waypoint_publisher',
            output='screen'
        )
    ])
