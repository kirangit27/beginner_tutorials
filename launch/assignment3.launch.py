import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():

    record_bag_arg = DeclareLaunchArgument('record_bag', default_value='false', description='Enable bag recording')
    
    talker_node = Node(
        package='beginner_tutorials', 
        executable='talker',
        name='minimal_publisher',
        output='screen',
        parameters=[{'publisher_frequency': launch.substitutions.LaunchConfiguration('freq', default=2.0)}],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    listener_node = Node(
        package='beginner_tutorials', 
        executable='listener',
        name='minimal_subscriber',
        output='screen'
    )

    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rosbag_record')),
        cmd=['ros2', 'bag', 'record', '-o', 'talker_rosbag', '-a'],
        output='screen'
    )

    rosbag_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=talker_node,
                on_start=[
                    recorder
                ]
            )
        )
   
    ld = LaunchDescription()
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(launch.actions.DeclareLaunchArgument('freq', default_value='2.0', description='Publisher frequency'))
    ld.add_action(rosbag_handler)
    ld.add_action(record_bag_arg)

    return ld
