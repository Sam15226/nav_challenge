from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '--render-engine',
                'ogre',
                '/home/samuel-salter/ros2_ws/src/nav_challenge/challenge.sdf'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
            output='screen'
        ),

        Node(
            package='nav_challenge',
            executable='navigator',
            output='screen'
        )
    ])
