import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge_args = []
    for i in range(1, 7):
        bridge_args.append(f'/model/drone{i}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose')

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )

    controller_node = Node(
        package='marl_controller',
        executable='controller_node',
        output='screen'
    )

    return LaunchDescription([
        ros_gz_bridge_node,
        controller_node
    ])
