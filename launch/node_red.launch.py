from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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

    # Include Rosbridge Launch File
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(FrontendLaunchDescriptionSource(rosbridge_dir + '/launch/rosbridge_websocket_launch.xml'), launch_arguments={'port':'9091'}.items())
    launch_description.add_action(rosbridge_launch)

    # Launch Description - Add Nodes
    launch_description.add_action(create_node_red_node())

    # Return Launch Description
    return launch_description
