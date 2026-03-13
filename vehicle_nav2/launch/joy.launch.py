from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'dev': '/dev/joy',  # 摇杆设备路径
                'deadzone': 0.1,          # 死区值，过滤微小抖动
                'autorepeat_rate': 20.0   # 发布频率（Hz）
            }]
        ),
        Node(
            package='joy_to_twist',
            executable='joy_to_twist_node',
            name='joy_to_twist_node'
        )
    ])
