from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laser2base = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_to_laser',
                    arguments=[
                        '--x', '0.22',
                        '--y', '0.0',
                        '--z', '0.0',
                        '--yaw', '0.0',
                        '--pitch', '0.0',
                        '--roll', '0.0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'laser'
                    ]
                  )        
    return LaunchDescription([
        laser2base,
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
