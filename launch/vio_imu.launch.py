import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def create_launch_description(context):
    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV name'
    )
    
    portname_arg = DeclareLaunchArgument(
        'portname',
        #default_value='/dev/ttyACM0',
        default_value='/dev/bluefox_imu',
        description='Port name for IMU device'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='460800',
        description='Serial port speed'
    )
    
    profiler_arg = DeclareLaunchArgument(
        'profiler',
        default_value=EnvironmentVariable('PROFILER', default_value='false'),
        description='Enable profiler'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose output'
    )
    
    debug_arg = DeclareLaunchArgument(
        'DEBUG',
        default_value='false',
        description='Enable debug mode'
    )
    
    desired_publish_rate_arg = DeclareLaunchArgument(
        'desired_publish_rate',
        default_value='200'
    )
    
    # Get launch configurations
    uav_name = LaunchConfiguration('UAV_NAME')
    portname = LaunchConfiguration('portname')
    baudrate = LaunchConfiguration('baudrate')
    profiler = LaunchConfiguration('profiler')
    verbose = LaunchConfiguration('verbose')
    custom_config = LaunchConfiguration('custom_config')
    debug = LaunchConfiguration('DEBUG')
    desired_publish_rate = LaunchConfiguration('desired_publish_rate')
    
    # Build parameters dictionary
    parameters = [
        {'uav_name': uav_name},
        {'enable_profiler': profiler},
        {'portname': portname},
        {'baudrate': baudrate},
        {'verbose': verbose},
        #{'use_timeout': True},
        #{'serial_rate': 460800}
        {'desired_publish_rate': desired_publish_rate},   # skip_rate = int(1000/desired_publish_rate)..... 100, 200, 500, 1000
    ]
    
    if custom_config.perform(context) != "":
        parameters += [
            custom_config
        ]
    
    vio_imu_node = ComposableNodeContainer(
        name='vio_imu_container',
        namespace=uav_name,
        package='rclcpp_components',
        executable='component_container',
        respawn=False,
        #prefix='xterm -e gdb -ex run --args',
        composable_node_descriptions=[
            ComposableNode(
                package='mrs_serial',
                plugin='vio_imu::VioImu',  # Assuming the nodelet is converted to a regular node
                name='vio_imu',
                namespace=uav_name,
                parameters=parameters,
                remappings=[
                    ('~/profiler', 'profiler'),
                    ('~/baca_protocol_out', '~/received_message'),
                    ('~/baca_protocol_in', '~/send_message'),
                    ('~/raw_in', '~/send_raw_message'),
                ],
            ),
        ]
    )
    
    return [
        uav_name_arg,
        portname_arg,
        baudrate_arg,
        profiler_arg,
        verbose_arg,
        debug_arg,
        desired_publish_rate_arg,
        vio_imu_node
    ]
    
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'custom_config',
            default_value='',
            description='Custom configuration file path'
        ),
        OpaqueFunction(function=create_launch_description)
    ])