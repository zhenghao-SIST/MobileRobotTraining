#!/usr/bin/env python
#-*- coding: utf-8 -*-
# Author: Zhenghao Li
# Email: lizhenghao@shanghaitech.edu.cn
# Institute: SIST
# Date: 2026-01-09

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64  # 或 Int32, 如果你需要整数指令
from chassis_controller.motor_driver import Driver 
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from rclpy.duration import Duration

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # 参数：轮距（单位：米），你可以根据实际底盘调整
        self.declare_parameter('wheel_base', 0.175)      # 两轮中心距离
        self.declare_parameter('wheel_radius', 0.045)   # 轮子半径（v->omega）
        self.declare_parameter('gear_ratio', 19)   # 轮子半径（v->omega）
        self.declare_parameter('left_direction', 1)   # 轮子半径（v->omega）
        self.declare_parameter('right_direction', 1)   # 轮子半径（v->omega）

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.left_direction = self.get_parameter('left_direction').value
        self.right_direction = self.get_parameter('right_direction').value

        # 订阅 cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        #self.scan  = self.create_subscription(
        #    LaserScan,
        #    '/scan',
        #    self.scan_cb,
        #    10
        #)
        self.driver = Driver('/dev/Motor')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.count = 0
        self.last_time = self.get_clock().now()


        self.timer = self.create_timer(0.05, self.update_odom)

        #Initialize Motor Driver

        #self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10)
        #self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10)

        self.get_logger().info('DiffDrive Controller started.')

    def scan_cb(self, msg):
        # --- D. 发送 TF 变换 ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp + rclpy.time.Duration(seconds=1.5)
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # 设置平移
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # 设置旋转 (将 Euler 角 th 转换为四元数)
        q = self.euler_to_quaternion(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp + rclpy.time.Duration(seconds=1.5)
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'

        # 设置平移
        t.transform.translation.x = 0.22 
        t.transform.translation.y = 0.0 
        t.transform.translation.z = 0.0

        # 设置旋转 (将 Euler 角 th 转换为四元数)
        q = self.euler_to_quaternion(0, 0, math.pi)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 更新时间

    def update_odom(self):
        current_time = self.get_clock().now()# + rclpy.time.Duration(seconds=0.5) 
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 计算时间间隔 delta_t

        if dt <= 0:
            return

        # --- A. 获取速度数据 ---
        v_l, v_r = self.driver.get_speed()

        v_l = float(v_l) * self.left_direction  / 60.0 * 2 * math.pi * self.wheel_radius / 19 
        v_r = float(v_r) * self.right_direction / 60.0 * 2 * math.pi * self.wheel_radius / 16

        # --- B. 差速运动学解算 ---
        v = (v_r + v_l) / 2.0             # 线速度
        w = (v_r - v_l) / self.wheel_base # 角速度 (rad/s)

        self.get_logger().info(f'Vel: {v}, {w}')

        # --- C. 积分计算位姿 (航位推算) ---
        delta_x = v * math.cos(self.th) * dt
        delta_y = v * math.sin(self.th) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        # --- D. 发送 TF 变换 ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # 设置平移
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0


        # 设置旋转 (将 Euler 角 th 转换为四元数)
        q = self.euler_to_quaternion(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        #t = TransformStamped()
        #t.header.stamp = current_time.to_msg()
        #t.header.frame_id = 'base_link'
        #t.child_frame_id = 'laser'

        #t.transform.translation.x = 0.22 
        #t.transform.translation.y = 0.0 
        #t.transform.translation.z = 0.0

        #q = self.euler_to_quaternion(0, 0, math.pi)
        #t.transform.rotation.x = q[0]
        #t.transform.rotation.y = q[1]
        #t.transform.rotation.z = q[2]
        #t.transform.rotation.w = q[3]

        #self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

    def cmd_vel_callback(self, msg):
        # 提取线速度和角速度
        try:
            vx = msg.linear.x      # 前进速度 (m/s)
            wz = msg.angular.z     # 旋转速度 (rad/s)

            # 差速底盘运动学：计算左右轮线速度
            L = self.wheel_base
            v_left = vx - (wz * L / 2.0)
            v_right = vx + (wz * L / 2.0)

            omega_left = v_left / self.wheel_radius
            omega_right = v_right / self.wheel_radius

            RPM_left = int(omega_left / (2 * math.pi) * 60 * 19 * self.left_direction)
            RPM_right = int(omega_right / (2 * math.pi) * 60 * 16 * self.right_direction)
            self.driver.set_speed(RPM_left, 0x01)
            self.driver.set_speed(RPM_right, 0x02)
            #self.driver.set_speed(RPM_right, 0x02)
            self.get_logger().info(f'Cmd: vx={vx:.2f}, wz={wz:.2f} → L={RPM_left}, R={RPM_right}')
        except Exception as e:
            self.get_logger().error(f'Write Error occur')
            self.get_logger().error(e)
            sys.exit(-1)


    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
