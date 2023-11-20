import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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

   
    ld = LaunchDescription()
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(launch.actions.DeclareLaunchArgument('freq', default_value='2.0', description='Publisher frequency'))

    return ld
