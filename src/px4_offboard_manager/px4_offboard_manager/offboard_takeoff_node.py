#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 自动 Offboard 起飞节点（ROS 2 + MAVROS2）
#
# 功能：
# 1. 等待 PX4 SITL + MAVROS + Odom 就绪
# 2. 预热：持续向 /mavros/setpoint_raw/attitude 发布 setpoint
# 3. 自动 arming（在当前模式下，例如 POSCTL）
# 4. ARM 成功后，再切 set_mode("OFFBOARD")
# 5. 在 Offboard 模式下从当前高度升到 takeoff_height
# 6. 抵达后悬停在目标高度（后续可在 HOVER 状态接 EGO+SO3）

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class OffboardTakeoffNode(Node):
    # ================= 状态机定义 =================
    STATE_WAIT_SIM_READY = 0      # 等 /mavros/state connected + /odom 就绪
    STATE_PREPARE_OFFBOARD = 1    # 预热：仅发 setpoint，不切模式
    STATE_WAIT_ARMED = 2          # 已发送 arming，等待 actual armed
    STATE_WAIT_OFFBOARD = 3       # 已 armed，等待 actual mode=OFFBOARD
    STATE_TAKEOFF_ASCEND = 4      # Offboard 下起飞到 2m
    STATE_HOVER = 5               # 悬停
    STATE_FAILSAFE = 6            # 故障

    def __init__(self):
        super().__init__('offboard_takeoff_node')

        # ================= 参数 =================
        # 可通过 ros2 param set 修改，例如：
        #   ros2 param set /offboard_takeoff_node hover_thrust 0.58
        self.declare_parameter('hover_thrust', 0.6)           # ★ 改成你标定好的 hover_thrust
        self.declare_parameter('takeoff_height', 2.0)         # 起飞目标高度（相对当前 z）
        self.declare_parameter('takeoff_kp', 1.0)             # 简易高度 P 控制增益
        self.declare_parameter('min_thrust', 0.2)             # 防止关油
        self.declare_parameter('max_thrust', 0.9)             # 防止顶满
        self.declare_parameter('offboard_prepare_time', 2.0)  # 预热时间（秒）
        self.declare_parameter('control_rate_hz', 50.0)       # 控制循环频率

        self.hover_thrust = self.get_parameter('hover_thrust').get_parameter_value().double_value
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        self.takeoff_kp = self.get_parameter('takeoff_kp').get_parameter_value().double_value
        self.min_thrust = self.get_parameter('min_thrust').get_parameter_value().double_value
        self.max_thrust = self.get_parameter('max_thrust').get_parameter_value().double_value
        self.offboard_prepare_time = self.get_parameter('offboard_prepare_time').get_parameter_value().double_value
        self.control_rate_hz = self.get_parameter('control_rate_hz').get_parameter_value().double_value

        self.get_logger().info(
            f'[OffboardTakeoffNode] hover_thrust={self.hover_thrust:.3f}, '
            f'takeoff_height={self.takeoff_height:.2f}, '
            f'control_rate={self.control_rate_hz:.1f} Hz'
        )

        # ================= 状态缓存 =================
        self.current_state = State()
        self.has_odom = False
        self.current_alt = 0.0
        self.takeoff_start_alt = None
        self.pre_offboard_start_time = None
        self.state = self.STATE_WAIT_SIM_READY

        # ================= QoS 配置 =================
        # MAVROS 的 /mavros/local_position/odom 一般是 BEST_EFFORT
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # /mavros/state 用默认可靠 QoS 即可
        state_qos_depth = 10

        # ================= 订阅 / 发布 =================
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            state_qos_depth
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_cb,
            odom_qos
        )

        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        # ================= 服务客户端 =================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        self.arming_client.wait_for_service()
        self.get_logger().info('Waiting for /mavros/set_mode service...')
        self.mode_client.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

        # ================= 定时控制循环 =================
        timer_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(timer_period, self.control_loop)

    # ----------------------------------------------------------------------
    # 回调：MAVROS State
    # ----------------------------------------------------------------------
    def state_cb(self, msg: State):
        self.current_state = msg

    # 回调：Odom
    def odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.current_alt = msg.pose.pose.position.z

    # ----------------------------------------------------------------------
    # 构造 AttitudeTarget：水平姿态 + 指定 thrust
    # ----------------------------------------------------------------------
    def build_attitude_target(self, thrust: float) -> AttitudeTarget:
        msg = AttitudeTarget()

        # 忽略 body_rate.x/y/z，只用 orientation + thrust
        msg.type_mask = 0b00000111

        # 水平姿态 + yaw = 0
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        msg.orientation = q

        # 忽略 body rates
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0

        thrust_clamped = float(max(self.min_thrust, min(self.max_thrust, thrust)))
        msg.thrust = thrust_clamped
        return msg

    # 悬停用
    def build_hover_attitude_target(self) -> AttitudeTarget:
        return self.build_attitude_target(self.hover_thrust)

    # 起飞用：简单高度 P 控制
    def build_takeoff_attitude_target(self, z_des: float, z_cur: float) -> AttitudeTarget:
        error = z_des - z_cur
        thrust = self.hover_thrust + self.takeoff_kp * error
        return self.build_attitude_target(thrust)

    # ----------------------------------------------------------------------
    # 调用 arming / set_mode（异步）
    # ----------------------------------------------------------------------
    def call_arming(self, arm: bool):
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)

        def done_cb(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f'Arming command success (arm={arm})')
                else:
                    # 这里你看到的就是 FAILED
                    self.get_logger().warn(f'Arming command FAILED (arm={arm})')
            except Exception as e:
                self.get_logger().error(f'Arming call failed: {e}')

        future.add_done_callback(done_cb)

    def call_set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)

        def done_cb(fut):
            try:
                res = fut.result()
                if res.mode_sent:
                    self.get_logger().info(f'Set_mode({mode}) sent successfully')
                else:
                    self.get_logger().warn(f'Set_mode({mode}) FAILED to send')
            except Exception as e:
                self.get_logger().error(f'Set_mode call failed: {e}')

        future.add_done_callback(done_cb)

    # ----------------------------------------------------------------------
    # 主控制循环：状态机
    # ----------------------------------------------------------------------
    def control_loop(self):
        # 0. 等仿真、MAVROS、ODOM就绪
        if self.state == self.STATE_WAIT_SIM_READY:
            if self.current_state.connected and self.has_odom:
                self.get_logger().info('PX4 connected and odom available, start OFFBOARD prepare...')
                self.pre_offboard_start_time = self.get_clock().now()
                self.state = self.STATE_PREPARE_OFFBOARD

        # 1. 预热：只发 hover setpoint，不改模式
        elif self.state == self.STATE_PREPARE_OFFBOARD:
            self.att_pub.publish(self.build_hover_attitude_target())
            now = self.get_clock().now()

            if now - self.pre_offboard_start_time >= Duration(seconds=self.offboard_prepare_time):
                self.get_logger().info('Offboard prepare done, sending ARM (without OFFBOARD)...')
                # 只发 ARM，不改模式
                self.call_arming(True)
                self.state = self.STATE_WAIT_ARMED

        # 2. 等待已 armed
        elif self.state == self.STATE_WAIT_ARMED:
            # 仍然保持 setpoint 流
            self.att_pub.publish(self.build_hover_attitude_target())

            if self.current_state.armed:
                self.get_logger().info(
                    f'Vehicle ARMED in mode={self.current_state.mode}, now switching to OFFBOARD...'
                )
                # 现在再切 OFFBOARD
                self.call_set_mode('OFFBOARD')
                self.state = self.STATE_WAIT_OFFBOARD

        # 3. 等待实际进入 OFFBOARD
        elif self.state == self.STATE_WAIT_OFFBOARD:
            self.att_pub.publish(self.build_hover_attitude_target())

            if self.current_state.mode == 'OFFBOARD':
                self.takeoff_start_alt = self.current_alt
                self.get_logger().info(
                    f'Entered OFFBOARD, start takeoff from z={self.takeoff_start_alt:.2f} m'
                )
                self.state = self.STATE_TAKEOFF_ASCEND

        # 4. Offboard 下起飞到目标高度
        elif self.state == self.STATE_TAKEOFF_ASCEND:
            if self.takeoff_start_alt is None:
                self.takeoff_start_alt = self.current_alt

            z_des = self.takeoff_start_alt + self.takeoff_height
            att = self.build_takeoff_attitude_target(z_des, self.current_alt)
            self.att_pub.publish(att)

            if abs(self.current_alt - z_des) < 0.1:
                self.get_logger().info(
                    f'Reached takeoff target height z={self.current_alt:.2f} m, switching to HOVER.'
                )
                self.state = self.STATE_HOVER

        # 5. 悬停：保持 hover_thrust
        elif self.state == self.STATE_HOVER:
            self.att_pub.publish(self.build_hover_attitude_target())
            # TODO: 后续在这里切换到轨迹跟踪状态（EGO+SO3）

        # 6. FAILSAFE：目前简单处理，啥都不发
        elif self.state == self.STATE_FAILSAFE:
            pass

        else:
            self.get_logger().error(f'Unknown state {self.state}, enter FAILSAFE.')
            self.state = self.STATE_FAILSAFE


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('OffboardTakeoffNode interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
