"""
This is a ROS2 python launch file that
loads the turtlebot3_burger URDF into a robot_state_publisher
and optionally allows viewing it in rviz

The argument use_rviz (true or false, default to true) controls whether rviz is launched
The argument use_jsp (true or false, default to true) controls whether joint_state_publisher is used to publish default joint states

The rviz configuration is stored in /config/basic_purple.rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='use_rviz', default_value='true', description='Launch rviz'),
        DeclareLaunchArgument(name='use_jsp', default_value='true', description='Launch joint_state_publisher'),
        
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             condition=LaunchConfigurationEquals('use_jsp', 'true')),
        
        Node(package="rviz2",
             executable="rviz2",
             condition=LaunchConfigurationEquals('use_rviz', 'true'),
             arguments=["-d", PathJoinSubstitution([FindPackageShare("turtlebot3_description"), "config", "basic_purple.rviz"])],),
    ])