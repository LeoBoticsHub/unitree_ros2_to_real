from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', 'test3_bag',  # Set the name to "my_custom_bag"
                '/evet'
                # '/imu/FL_data', '/imu/FR_data',          # Topics to record
                # '/imu/RL_data', '/imu/RR_data',
                # '/imu/trunk_data', '/joint_group_effort_controller/joint_trajectory',
                # '/odom/ground_truth'
            ],
            output='screen'
        )
    ])
