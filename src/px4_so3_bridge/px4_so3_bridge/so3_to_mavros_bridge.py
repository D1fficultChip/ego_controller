#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from quadrotor_msgs.msg import SO3Command
from mavros_msgs.msg import AttitudeTarget


class So3ToMavrosBridge(Node):
    def __init__(self):
        super().__init__('so3_to_mavros_bridge')

        # 标定参数：先用 mass*g ≈ 15N + 悬停油门 0.5
        # 之后可以改成参数/yaml，这里先写死调通再说
        self.hover_force = 15.0      # N，大约等于 1.535kg * 9.81
        self.hover_thrust = 2.0   # PX4 上悬停油门的粗略值
        self.min_thrust = 0.25       # 保底，避免一切就掉
        self.max_thrust = 2.5       # 保顶，避免一下子拉太猛

        self.get_logger().info(
            f"So3ToMavrosBridge: hover_force={self.hover_force:.2f} N, "
            f"hover_thrust={self.hover_thrust:.2f}, "
            f"thrust_limits=[{self.min_thrust:.2f}, {self.max_thrust:.2f}]"
        )

        # 订阅 SO3 控制器输出
        self.create_subscription(
            SO3Command,
            '/so3_cmd',
            self.so3_cmd_cb,
            10
        )

        # 发布到 PX4 的 attitude setpoint
        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

    def so3_cmd_cb(self, msg: SO3Command):
        att = AttitudeTarget()
        att.header.stamp = self.get_clock().now().to_msg()
        att.header.frame_id = 'map'

        # 1) 姿态直接跟 SO3 的期望
        att.orientation = msg.orientation

        # 2) 力 -> thrust：用总力模长标定
        fx = msg.force.x
        fy = msg.force.y
        fz = msg.force.z
        f_norm = math.sqrt(fx * fx + fy * fy + fz * fz)

        if self.hover_force > 1e-3:
            scale = f_norm / self.hover_force
        else:
            scale = 1.0

        thrust_raw = self.hover_thrust * scale

        # 3) 限幅，避免一次给太小/太大的油门
        thrust = max(self.min_thrust, min(self.max_thrust, thrust_raw))
        att.thrust = float(thrust)

        # 4) 忽略 body rate，让 PX4 自己管角速度
        att.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )

        self.att_pub.publish(att)


def main(args=None):
    rclpy.init(args=args)
    node = So3ToMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
