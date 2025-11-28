#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        
        # 订阅你的真实里程计话题
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_world',  # 确保这个和你的 Bridge 发出的话题一致
            self.handle_odom,
            10)
        self.get_logger().info("Odom -> TF Broadcaster Started! Listening to /odom_world")

    def handle_odom(self, msg):
        t = TransformStamped()

        # 这一步非常关键：连接 world -> base_link
        # 这样规划器就能通过 world -> base_link -> camera_link 找到相机的位姿
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'       # 父坐标系 (EGO 的世界系)
        t.child_frame_id = 'base_link'    # 子坐标系 (飞机本体)

        # 搬运位置
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 搬运姿态 (四元数)
        t.transform.rotation = msg.pose.pose.orientation

        # 发送 TF
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
