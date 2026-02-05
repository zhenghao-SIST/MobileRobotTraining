#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration


class ScanTFTimeChecker(Node):
    def __init__(self):
        super().__init__('scan_tf_time_checker')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        self.get_logger().info('Scan-TF time checker started.')

    def scan_cb(self, msg: LaserScan):
        now = self.get_clock().now()
        scan_time = Time.from_msg(msg.header.stamp)

        # 1️⃣ scan vs now
        dt_scan_now = (now - scan_time).nanoseconds * 1e-9

        log = []
        log.append(f'[SCAN]   frame={msg.header.frame_id}')
        log.append(f'[TIME]   now - scan = {dt_scan_now:+.3f} s')

        # 2️⃣ TF lookup at scan time
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                scan_time,
                timeout=Duration(seconds=0.1)
            )

            tf_time = Time.from_msg(tf.header.stamp)

            dt_tf_scan = (tf_time - scan_time).nanoseconds * 1e-9
            dt_tf_now = (now - tf_time).nanoseconds * 1e-9

            log.append(f'[TF]     FOUND')
            log.append(f'[TIME]   tf - scan = {dt_tf_scan:+.3f} s')
            log.append(f'[TIME]   now - tf   = {dt_tf_now:+.3f} s')

        except Exception as e:
            log.append(f'[TF]     NOT FOUND at scan time')
            log.append(f'[ERROR]  {str(e).splitlines()[0]}')

        self.get_logger().info(' | '.join(log))


def main():
    rclpy.init()
    node = ScanTFTimeChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
