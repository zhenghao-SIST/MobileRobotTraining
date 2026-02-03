# your_robot_nav2/launch/nav2_minimal.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置路径
    pkg_dir = get_package_share_directory('your_robot_nav2')
    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2.yaml')

    # 声明 use_sim_time（真机设为 false）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # 设置环境变量（可选）
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # 启动 Nav2 核心
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_file}]
        ),

        # AMCL 定位
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        # 控制器服务器（含局部规划）
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # 规划器服务器（全局路径）
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 恢复行为（简单避障恢复）
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # BT 导航器（主入口）
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 生命周期管理器（自动激活所有节点）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator'
                        ]}]
        ),

        # RViz 可视化（可选，真机可注释掉）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2.rviz')],
            output='screen',
            condition=condition=IfCondition('False')
        )
    ])
