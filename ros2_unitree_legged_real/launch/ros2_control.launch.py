from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Declaring launch arguments
    publish_odom_tf = DeclareLaunchArgument('publish_odom_tf', default_value='False')
    robot_type = DeclareLaunchArgument('robot_type', default_value='b1')

    # Convert robot name in python string
    robot_name = LaunchConfiguration('robot_type').perform(context)

    # Launch ros2 control node
    ros2_control = Node(
        package='ros2_unitree_legged_real',
        executable='unitree_high_ros2_control',
        name='unitree_high_ros2_control',
        output='screen',
        parameters=[{'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
                     'robot_name': robot_name},
                     os.path.join(get_package_share_directory("ros2_unitree_legged_real"), 'config', robot_name + '.yaml')]
    )

    print(os.path.join(get_package_share_directory("ros2_unitree_legged_real"), 'config', robot_name + '.yaml'))

    # upload robot description for the robot state publisher
    robot_description = get_package_share_directory(robot_name + '_description')
    upload_launch = TimerAction(period=1.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description + '/launch/upload.launch.py'), 
        launch_arguments = {
            'use_rviz': 'false'
        }.items()
    )])


    return [publish_odom_tf, robot_type, ros2_control, upload_launch]

def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])