from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def create_launch_description(context):
    uav_name_arg = DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV name'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='serial_uvdar',
        description='Name given to the BacaProtocol node instance'
    )

    portname_arg = DeclareLaunchArgument(
        'portname',
        default_value='/dev/MRS_MODULE1',
        description='Serial port the board is connected to'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port speed'
    )

    standalone_arg = DeclareLaunchArgument(
        'standalone',
        default_value='true'
    )

    container_id_arg = DeclareLaunchArgument(
        'container_id',
        default_value='',
    )

    uav_name = LaunchConfiguration('UAV_NAME')
    node_name = LaunchConfiguration('node_name')
    portname = LaunchConfiguration('portname')
    baudrate = LaunchConfiguration('baudrate')

    parameters = [
        {'uav_name': uav_name},
        {'portname': portname},
        {'baudrate': baudrate},
    ]

    baca_node = ComposableNode(
        package='mrs_serial',
        plugin='baca_protocol::BacaProtocol',
        name=node_name,
        namespace=uav_name,
        parameters=parameters,
        remappings=[
            # subscriber: LED manager (and other clients) publish outbound frames here.
            # NOTE: a relative remap target resolves against the node's *namespace*, not
            # its name (unlike ROS1's '~'), so the node name has to be included explicitly
            # here to land on .../<node_name>/send_message.
            ('~/baca_protocol_in', [node_name, '/send_message']),
            # publisher: frames received back from the board
            ('~/baca_protocol_out', [node_name, '/received_message']),
        ],
    )

    loader = LoadComposableNodes(
        condition=UnlessCondition(LaunchConfiguration('standalone')),
        composable_node_descriptions=[baca_node],
        target_container=LaunchConfiguration('container_id'),
    )

    baca_container = ComposableNodeContainer(
        condition=IfCondition(LaunchConfiguration('standalone')),
        name='baca_protocol_container',
        namespace=uav_name,
        package='rclcpp_components',
        executable='component_container',
        respawn=False,
        composable_node_descriptions=[baca_node]
    )

    return [
        uav_name_arg,
        node_name_arg,
        portname_arg,
        baudrate_arg,
        standalone_arg,
        container_id_arg,
        loader,
        baca_container,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=create_launch_description)
    ])
