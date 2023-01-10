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
from launch.substitutions import TextSubstitution
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='use_rviz', default_value='true', description='Launch rviz'),
        DeclareLaunchArgument(name='use_jsp', default_value='true', description='Launch joint_state_publisher'),
        DeclareLaunchArgument(name='color',
                              default_value='purple',
                              description='Color of the turtlebot base',
                              choices=['purple', 'red', 'green', 'blue', '']),

        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             arguments=[PathJoinSubstitution([FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"])],
             condition=LaunchConfigurationEquals('use_jsp', 'true'),
             namespace=LaunchConfiguration('color')),
        
        Node(package="rviz2",
             executable="rviz2",
             condition=LaunchConfigurationEquals('use_rviz', 'true'),
             arguments=["-d", [PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                    "config",
                                                    "basic_"]),
							   LaunchConfiguration('color'),
							   TextSubstitution(text=".rviz")]],
             namespace=LaunchConfiguration('color')),
        
        Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             namespace=LaunchConfiguration('color'),
             parameters=[
               {"robot_description":
                Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"]),
                          TextSubstitution(text=" color:="),
                          LaunchConfiguration('color'),])},
               {"frame_prefix":
                [LaunchConfiguration('color'),
                 TextSubstitution(text="/")]}
            ]
            ),
    ])