from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ros2_control = Node(
        package='ros2_unitree_legged_real',
        executable='unitree_high_ros2_control',
        name='unitree_high_ros2_control',
        output='screen'
    )

    b1_description = get_package_share_directory('b1_description')
    upload_launch = TimerAction(period=1.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(b1_description + '/launch/upload.launch.py'), 
        launch_arguments = {
            'use_rviz': 'false'
        }.items()
    )])
    
    return LaunchDescription([ros2_control, upload_launch])