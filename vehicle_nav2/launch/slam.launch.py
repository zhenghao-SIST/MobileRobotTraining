from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件路径（使用 PathJoinSubstitution + FindPackageShare 更规范）
    config_dir = PathJoinSubstitution([
        get_package_share_directory('vehicle_nav2'),
        'config'
    ])

    rviz_config_file = PathJoinSubstitution([
        config_dir,
        'slam_rviz.rviz'
    ])

    params_file = PathJoinSubstitution([
        config_dir,
        'mapper_params_online_async.yaml'
    ])

    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 启动 SLAM Toolbox 异步建图节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'mode': 'mapping'}
        ]
    )

    return LaunchDescription([
        rviz_node,
        slam_toolbox_node
    ])
