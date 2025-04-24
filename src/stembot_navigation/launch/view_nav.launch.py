from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'stembot_navigation'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'navigation_launch.rviz'),
        description='Location of RViz config file'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(rviz)

    return ld