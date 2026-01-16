from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_controller',
            executable='chassis_controller',
            name='chassis_controller',
            output='screen',
            parameters=[
                {
                    'wheel_base': 0.175,
                    'wheel_radius': 0.045,
                    'left_direction': -1,
                    'right_direction': 1,
                    'gear_ratio': 19 
                }
            ]
        )
    ])
