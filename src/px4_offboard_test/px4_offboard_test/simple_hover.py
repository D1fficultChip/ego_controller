#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

import time


class SimpleOffboardHover(Node):
    def __init__(self):
        super().__init__('simple_offboard_hover')

        # 当前飞控状态 & 当前位置
        self.current_state = None
        self.current_pose = None
        self.target_pose = None

        # 订阅飞控状态
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

# 模拟 rclcpp::SensorDataQoS()
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_cb,
            sensor_qos
        )


        # 发布位置 setpoint
        self.sp_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # 20Hz 发布 setpoint
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.start_time = time.time()
        self.got_initial_pose = False

        self.get_logger().info("SimpleOffboardHover node started. Wait for odom...")

    def state_cb(self, msg: State):
        self.current_state = msg

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg

        # 第一次收到位置时，设置目标为 “当前位置的上方 2m”
        if not self.got_initial_pose:
            x0 = msg.pose.pose.position.x
            y0 = msg.pose.pose.position.y
            z0 = msg.pose.pose.position.z

            target = PoseStamped()
            target.header.frame_id = 'map'  # 或 'local_origin' 看你 px4 配置，一般 'map' 也行
            target.pose.position.x = x0
            target.pose.position.y = y0
            target.pose.position.z = z0 + 2.0

            # 姿态我们先保持默认（四元数 0,0,0,1）
            target.pose.orientation.w = 1.0

            self.target_pose = target
            self.got_initial_pose = True

            self.get_logger().info(
                f"Initial pose: ({x0:.2f}, {y0:.2f}, {z0:.2f}), "
                f"target set to z+2m -> ({x0:.2f}, {y0:.2f}, {z0+2.0:.2f})"
            )

    def timer_cb(self):
        # 只有在有了目标点之后才开始发 setpoint
        if self.target_pose is None:
            return

        # 更新时间戳
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(self.target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOffboardHover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

