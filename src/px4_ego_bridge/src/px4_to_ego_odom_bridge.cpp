#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Px4ToEgoOdomBridge : public rclcpp::Node
{
public:
  Px4ToEgoOdomBridge() : Node("px4_to_ego_odom_bridge")
  {
    // MAVROS 是 SensorDataQoS（best effort），订阅端也要用这个
    auto sensor_qos = rclcpp::SensorDataQoS();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom",
      sensor_qos,
      std::bind(&Px4ToEgoOdomBridge::odomCallback, this, std::placeholders::_1));

    odom_world_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_world", 10);
    grid_map_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/grid_map/odom", 10);

    RCLCPP_INFO(this->get_logger(),
      "Px4ToEgoOdomBridge initialized: /mavros/local_position/odom -> /odom_world, /grid_map/odom");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom_world = *msg;
    // 统一 frame_id，RViz 和 EGO 都用这个
    odom_world.header.frame_id = "world";   // 你也可以用 "map"，但要和 RViz Fixed Frame 对上

    nav_msgs::msg::Odometry grid_map_odom = odom_world;

    odom_world_pub_->publish(odom_world);
    grid_map_odom_pub_->publish(grid_map_odom);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_world_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr grid_map_odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4ToEgoOdomBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
