#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import SO3Command
from mavros_msgs.msg import AttitudeTarget

class So3ToMavrosBridge(Node):
    def __init__(self):
        super().__init__('so3_to_mavros_bridge')

        # ==========================================
        # 关键参数配置
        # ==========================================
        # 1. 这里必须和你 C++ so3_control 里的 mass 参数严格对应！
        # 如果 C++ 里 mass=1.535, 这里就要写 15.0 (1.535 * 9.8)
        # 如果 C++ 里 mass=1.0,   这里就要写 9.8  (1.0 * 9.8)
        self.mass_kg = 1.535  # 请根据实际情况修改！
        self.g = 9.81
        self.hover_force = self.mass_kg * self.g 
        
        # 2. 悬停油门标定值 (Hover Thrust)
        # 你的目标就是调整这个值，让飞机稳住
        # 先给 0.55 (Iris 典型值)，不要给 0.8，0.8 太危险了
        self.hover_thrust = 0.705 
        
        # 3. 安全限幅 (Thrust 是归一化的 0.0 ~ 1.0)
        self.min_thrust = 0.10       # 稍微给点怠速
        self.max_thrust = 1.0        # 绝对不能超过 1.0！

        self.get_logger().info(
            f">>> Bridge Start: Mass={self.mass_kg}kg, Ref_Force={self.hover_force:.2f}N, "
            f"Hover_Thrust={self.hover_thrust}"
        )

        self.create_subscription(SO3Command, '/so3_cmd', self.so3_cmd_cb, 10)
        self.att_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

    def so3_cmd_cb(self, msg: SO3Command):
        att = AttitudeTarget()
        att.header.stamp = self.get_clock().now().to_msg()
        att.header.frame_id = 'map' # MAVROS 其实不看这个 frame_id，但写上比较好

        # 1. 姿态透传 (Quaternion)
        att.orientation = msg.orientation

        # 2. 力映射 (Force -> Thrust 0..1)
        fx = msg.force.x
        fy = msg.force.y
        fz = msg.force.z
        f_norm = math.sqrt(fx*fx + fy*fy + fz*fz)

        # 避免除以 0
        if self.hover_force > 1e-3:
            # 核心公式：(当前力 / 悬停力) * 悬停油门
            scale = f_norm / self.hover_force
            thrust_raw = self.hover_thrust * scale
        else:
            thrust_raw = 0.0

        # 3. 严格限幅 [0.0, 1.0]
        thrust = max(self.min_thrust, min(self.max_thrust, thrust_raw))
        att.thrust = float(thrust)

        # 4. 设置掩码 (忽略角速度)
        att.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )

        self.att_pub.publish(att)

        # ==========================================
        # 5. DEBUG 日志 (很重要！调试完再注释掉)
        # ==========================================
        # 观察 Input Force 和 Output Thrust 的关系
        # 如果 Force 只有 10N，说明 C++ 那边 Mass 设小了
        # 如果 Thrust 只有 0.25，说明触发了 min_thrust
        # self.get_logger().info(
        #    f"In_Force: {f_norm:.2f}N | Ref_Force: {self.hover_force:.2f}N | "
        #    f"Scale: {scale:.2f} | Out_Thrust: {thrust:.3f}",
        #    throttle_duration_sec=0.5 # 每0.5秒打印一次，防止刷屏
        # )

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