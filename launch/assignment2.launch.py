# Import necessary Python modules
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the talker node with the frequency parameter
    talker_node = Node(
        package='beginner_tutorials',  # Replace with your actual package name
        executable='talker',
        name='minimal_publisher',
        output='screen',
        parameters=[{'publisher_frequency': launch.substitutions.LaunchConfiguration('freq', default=2.0)}]
    )

    # Define the listener node
    listener_node = Node(
        package='beginner_tutorials',  # Replace with your actual package name
        executable='listener',
        name='minimal_subscriber',
        output='screen'
    )

    # Create the launch description and add the talker and listener nodes
    ld = LaunchDescription()

    # Add the talker node with the 'freq' argument
    ld.add_action(talker_node)

    # Add the listener node
    ld.add_action(listener_node)

    # Declare the 'freq' argument with a default value
    ld.add_action(launch.actions.DeclareLaunchArgument('freq', default_value='2.0', description='Publisher frequency'))

    return ld
