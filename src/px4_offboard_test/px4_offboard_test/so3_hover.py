#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand  # ego 仓库里的消息包


class So3Hover(Node):
    def __init__(self):
        super().__init__('so3_hover')

        self.current_odom = None
        self.target_cmd = None
        self.got_initial = False

        # QoS 模拟 SensorDataQoS，避免之前那种 RELIABILITY 警告
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_cb,
            sensor_qos
        )

        self.cmd_pub = self.create_publisher(
            PositionCommand,
            '/drone_0_planning/pos_cmd',
            10
        )

        # 20Hz 发布
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info("So3Hover started. Waiting for odom...")

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

        if not self.got_initial:
            x0 = msg.pose.pose.position.x
            y0 = msg.pose.pose.position.y
            z0 = msg.pose.pose.position.z

            cmd = PositionCommand()
            cmd.position.x = x0
            cmd.position.y = y0
            cmd.position.z = z0 + 2.0   # 目标：当前高度 +2m

            # 速度 / 加速度设 0，留给 SO3 自己的 PD
            cmd.velocity.x = 0.0
            cmd.velocity.y = 0.0
            cmd.velocity.z = 0.0

            cmd.acceleration.x = 0.0
            cmd.acceleration.y = 0.0
            cmd.acceleration.z = 0.0

            # yaw 固定当前 yaw（简单处理：不从四元数里提了，先用 0 也行）
            cmd.yaw = 0.0
            cmd.yaw_dot = 0.0

            # kx/kv 全 0，避免覆盖你在 SO3 里整定好的增益
            cmd.kx = [0.0, 0.0, 0.0]
            cmd.kv = [0.0, 0.0, 0.0]

            self.target_cmd = cmd
            self.got_initial = True

            self.get_logger().info(
                f"Initial z={z0:.2f}, hover target set to z+2m -> {z0+2.0:.2f}"
            )

    def timer_cb(self):
        if not self.got_initial or self.target_cmd is None:
            return
        self.target_cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(self.target_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = So3Hover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

