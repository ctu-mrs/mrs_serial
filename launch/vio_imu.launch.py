import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME', default_value='uav'),
        description='UAV name'
    )
    
    portname_arg = DeclareLaunchArgument(
        'portname',
        #default_value='/dev/vio_imu',
        default_value='/dev/ttyACM0',
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
    
    custom_config_arg = DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='Custom configuration file path'
    )
    
    debug_arg = DeclareLaunchArgument(
        'DEBUG',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Get launch configurations
    uav_name = LaunchConfiguration('UAV_NAME')
    portname = LaunchConfiguration('portname')
    baudrate = LaunchConfiguration('baudrate')
    profiler = LaunchConfiguration('profiler')
    verbose = LaunchConfiguration('verbose')
    custom_config = LaunchConfiguration('custom_config')
    debug = LaunchConfiguration('DEBUG')
    
    # Build parameters dictionary
    parameters = [
        {'uav_name': uav_name},
        {'enable_profiler': profiler},
        {'portname': portname},
        {'baudrate': baudrate},
        {'verbose': verbose},
        #{'use_timeout': True},
        #{'serial_rate': 460800}
    ]
    
    # Add default config file
    # mrs_serial_config = os.path.join(
    #     get_package_share_directory('mrs_serial'),
    #     'config',
    #     'imu_default.yaml'
    # )
    # parameters.append(mrs_serial_config)
    
    # Add custom config conditionally
    # custom_config_param = PythonExpression([
    #     '"', custom_config, '" if "', custom_config, '" != "" else ""'
    # ])
    
    # # Create the node
    # vio_imu_node = Node(
    #     package='mrs_serial',  # Assuming this is the package name for the nodelet
    #     executable='MrsSerial_VioImu',  # ROS2 executable name
    #     name='vio_imu',
    #     namespace=uav_name,
    #     parameters=parameters + [
    #         # Add custom config if provided
    #         PythonExpression([
    #             '{"custom_config": "', custom_config, '"}',
    #             ' if "', custom_config, '" != "" else {}'
    #         ])
    #     ],
    #     remappings=[
    #         ('~/profiler', 'profiler'),
    #         ('~/baca_protocol_out', '~/received_message'),
    #         ('~/baca_protocol_in', '~/send_message'),
    #         ('~/raw_in', '~/send_raw_message'),
    #     ],
    #     output='screen',
    #     #prefix="xterm -e gdb -ex=r --args",
    #     condition=IfCondition(PythonExpression([
    #         'True'  # Always launch in ROS2, nodelet concept doesn't apply
    #     ]))
    # )
    
    # parameters += [
    #     # Add custom config if provided
    #     PythonExpression([
    #         '{"custom_config": "', custom_config, '"}',
    #         ' if "', custom_config, '" != "" else ""'
    #     ])
    # ],
    
    # if _custom_config_file != '':
    #     print("appending params")
    #     parameters.append(_custom_config_file)
    
    vio_imu_node = ComposableNodeContainer(
        name='vio_imu_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        respawn=False,
        #prefix='xterm -e gdb -ex run --args',
        #arguments=['--ros-args', '--log-level', "info"],
        composable_node_descriptions=[
            ComposableNode(
                package='mrs_serial',
                plugin='vio_imu::VioImu',  # Assuming the nodelet is converted to a regular node
                name='vio_imu',
                namespace=uav_name,
                #output='screen',
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
    
    return LaunchDescription([
        uav_name_arg,
        portname_arg,
        baudrate_arg,
        profiler_arg,
        verbose_arg,
        custom_config_arg,
        debug_arg,
        vio_imu_node
    ])
