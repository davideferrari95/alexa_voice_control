from launch import LaunchDescription
from launch_ros.actions import Node

def create_node_red_node():

    # NodeRED - Node
    node = Node(
        package='alexa_voice_control', executable='node_red.py', name='node_red',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')]
    )

    return node

def generate_launch_description():

    # Launch Description
    launch_description = LaunchDescription()

    # Launch Description - Add Nodes
    launch_description.add_action(create_node_red_node())

    # Return Launch Description
    return launch_description
