from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    waypoint_publisher = Node(
        package='traj_planner',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(waypoint_publisher)

    return ld