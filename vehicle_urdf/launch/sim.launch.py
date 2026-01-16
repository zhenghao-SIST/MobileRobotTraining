#!/usr/bin/env python
#-*- coding: utf-8 -*-
# Author: Zhenghao Li
# Email: lizhenghao@shanghaitech.edu.cn
# Institute: SIST
# Date: 2026-01-07
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro


package_name = 'vehicle_urdf'
pkg_share = get_package_share_directory(package_name)
# 获取功能包在 install 目录下的上级路径，这样 Gazebo 就能在 /install 目录下找到所有包
pkg_install_path = os.path.join(get_package_prefix(package_name), 'share')

# 设置环境变量，让 Gazebo 搜索到 meshes
if 'GZ_SIM_RESOURCE_PATH' in os.environ:
    gz_sim_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + pkg_install_path
else:
    gz_sim_resource_path = pkg_install_path


def generate_launch_description():
    # 1. 处理 URDF (Xacro) 文件
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'vehicle.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 2. 创建 robot_state_publisher 节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True}]
    )

    # 3. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), # -r 表示自动运行，不暂停
    )

    # 4. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'diff_vehicle', '-z', '0.5'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_sim_resource_path),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
