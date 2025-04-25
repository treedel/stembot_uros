import os

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'stembot_bringup'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = 'false'
    use_sim_time_bool = (use_sim_time == 'true')

    use_uros_agent = LaunchConfiguration('use_uros_agent')
    uros_serial_port = LaunchConfiguration('uros_serial_port')
    uros_serial_baudrate = LaunchConfiguration('uros_serial_baudrate')
    use_lidar = LaunchConfiguration('use_lidar')
    use_range_sensor = LaunchConfiguration('use_range_sensor')
    use_camera = LaunchConfiguration('use_camera')
    camera_device = LaunchConfiguration('camera_device')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')
    use_jsp = LaunchConfiguration('use_jsp')

    declare_use_uros_agent = DeclareLaunchArgument(
        name='use_uros_agent',
        default_value='true',
        description='Whether to launch micro ros agent'
    )

    declare_uros_serial_port = DeclareLaunchArgument(
        name='uros_serial_port', 
        default_value='/dev/ttyACM0',
        description='The serial communication port for micro ros'
    )

    declare_uros_serial_baudrate = DeclareLaunchArgument(
        name='uros_serial_baudrate', 
        default_value='115200',
        description='The serial communication baud rate for micro ros'
    )

    declare_use_lidar = DeclareLaunchArgument(
        name='use_lidar',
        default_value='true',
        description='Whether to use lidar'
    )

    declare_use_range_sensor = DeclareLaunchArgument(
        name='use_range_sensor',
        default_value='true',
        description='Whether to use range sensor'
    )

    declare_use_camera = DeclareLaunchArgument(
        name='use_camera',
        default_value='false',
        description='Whether to use Camera'
    )

    declare_camera_device = DeclareLaunchArgument(
        name='camera_device',
        default_value='0',
        description='Camera device number to identify the particular camera'
    )

    declare_use_jsp = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        description='Whether to launch joint state publisher'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'bringup_launch.rviz'),
        description='Location of RViz config file'
    )

    micro_ros_agent = Node(
        condition=IfCondition(use_uros_agent),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', uros_serial_port, '--baudrate', uros_serial_baudrate]
    )

    package_sllidar_ros2 = FindPackageShare(package='sllidar_ros2').find('sllidar_ros2')
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_sllidar_ros2, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'frame_id': 'lidar_link',
        }.items(),
        condition=IfCondition(use_lidar)
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{
            'gain': 0.01,
            'use_sim_time': use_sim_time_bool,
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
            "fixed_frame": "odom"
        }],
        remappings=[
            ('/imu/data_raw', '/imu_data_raw'),
            ('/imu/data', '/imu_filtered'),
        ]
    )

    ekf_param_file = os.path.join(package_share, 'config', 'ekf_hw.yaml')
    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time_bool}
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
    )

    ultra_f1 = Node(
        condition=IfCondition(use_range_sensor),
        package='hcsr04',
        executable='hcsr04_driver',
        parameters=[{
            'trig': 17,
            'echo': 18,
            'publish_frequency': 5.0,
            'frame_id': 'ultra_f1_link'
        }],
        remappings=[
            ('/range', '/range_ultra_f1')
        ]
    )

    ultra_f2 = Node(
        condition=IfCondition(use_range_sensor),
        package='hcsr04',
        executable='hcsr04_driver',
        parameters=[{
            'trig': 23,
            'echo': 24,
            'publish_frequency': 5.0,
            'frame_id': 'ultra_f2_link'
        }],
        remappings=[
            ('/range', '/range_ultra_f2')
        ]
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'description.launch.py')),
            launch_arguments={
                'use_rviz': use_rviz,
                'rviz_config_path': rviz_config_path,
                'use_jsp': use_jsp,
                'use_jsp_gui': 'false',
                'use_sim_time': use_sim_time,
        }.items()
    )

    camera = Node(
        condition=IfCondition(use_camera),
        package='image_tools',
        executable='cam2image',
        output='screen',
        parameters=[{'camera_device': camera_device}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_uros_agent)
    ld.add_action(declare_uros_serial_port)
    ld.add_action(declare_uros_serial_baudrate)
    ld.add_action(declare_use_lidar)
    ld.add_action(declare_use_range_sensor)
    ld.add_action(declare_use_camera)
    ld.add_action(declare_camera_device)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(declare_use_jsp)

    ld.add_action(micro_ros_agent)
    ld.add_action(lidar)
    ld.add_action(imu_filter)
    ld.add_action(ekf_localization)
    ld.add_action(ultra_f1)
    ld.add_action(ultra_f2)
    ld.add_action(robot_description)
    ld.add_action(camera)
    

    return ld