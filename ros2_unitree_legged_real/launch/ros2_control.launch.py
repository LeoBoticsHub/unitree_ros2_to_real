from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    publish_odom_tf = DeclareLaunchArgument('publish_odom_tf', default_value='False')

    ros2_control = Node(
        package='ros2_unitree_legged_real',
        executable='unitree_high_ros2_control',
        name='unitree_high_ros2_control',
        output='screen',
        parameters=[{'publish_odom_tf': LaunchConfiguration('publish_odom_tf')}]
    )

    b1_description = get_package_share_directory('b1_description')
    upload_launch = TimerAction(period=1.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(b1_description + '/launch/upload.launch.py'), 
        launch_arguments = {
            'use_rviz': 'false'
        }.items()
    )])
    
    return LaunchDescription([publish_odom_tf, ros2_control, upload_launch])