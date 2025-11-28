#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// === 全局状态：用于 PX4 <-> EGO 世界对齐 ===
namespace {
  // 当前 PX4 位姿（由 odom_callback 维护）
  Eigen::Vector3d g_current_pos(0.0, 0.0, 0.0);
  double g_current_yaw = 0.0;
  bool g_has_odom = false;

  // 第一次 PositionCommand 的 offset（EGO 世界 -> PX4 世界）
  bool g_has_offset = false;
  Eigen::Vector3d g_pos_offset(0.0, 0.0, 0.0);
  double g_yaw_offset = 0.0;
}

class SO3ControlComponent : public rclcpp::Node
{
public:
    SO3ControlComponent(const rclcpp::NodeOptions &options)
        : Node("SO3ControlComponent", options), position_cmd_updated_(false), position_cmd_init_(false), des_yaw_(0), des_yaw_dot_(0), current_yaw_(0), enable_motors_(true), // FIXME
          use_external_yaw_(false)
    {
        onInit();
    }

    void onInit(void);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void publishSO3Command(void);
    void position_cmd_callback(const quadrotor_msgs::msg::PositionCommand::ConstPtr &cmd);
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom);
    void enable_motors_callback(const std_msgs::msg::Bool::ConstPtr &msg);
    void corrections_callback(const quadrotor_msgs::msg::Corrections::ConstPtr &msg);
    void imu_callback(const sensor_msgs::msg::Imu &imu);

    SO3Control controller_;
    rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    bool position_cmd_updated_, position_cmd_init_;
    std::string frame_id_;

    Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
    double des_yaw_, des_yaw_dot_;
    double current_yaw_;
    bool enable_motors_;
    bool use_external_yaw_;
    double kR_[3], kOm_[3], corrections_[3];
    double init_x_, init_y_, init_z_;
};

void SO3ControlComponent::publishSO3Command(void)
{   
    // std::cout<< "pub so3 cmd!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                                 des_yaw_dot_, kx_, kv_);

    const Eigen::Vector3d &force = controller_.getComputedForce();
    const Eigen::Quaterniond &orientation = controller_.getComputedOrientation();

    auto so3_command = std::make_shared<quadrotor_msgs::msg::SO3Command>(); // 使用智能指针

    // 设置消息内容
    so3_command->header.stamp = this->get_clock()->now(); // ROS 2 使用 get_clock() 获取当前时间
    so3_command->header.frame_id = frame_id_;
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();

    // 填充 kR 和 kOm
    for (int i = 0; i < 3; i++)
    {
        so3_command->kr[i] = kR_[i];
        so3_command->kom[i] = kOm_[i];
    }

    // 填充辅助信息
    so3_command->aux.current_yaw = current_yaw_;
    so3_command->aux.kf_correction = corrections_[0];
    so3_command->aux.angle_corrections[0] = corrections_[1];
    so3_command->aux.angle_corrections[1] = corrections_[2];
    so3_command->aux.enable_motors = enable_motors_;
    so3_command->aux.use_external_yaw = use_external_yaw_;

    // 发布消息
    so3_command_pub_->publish(*so3_command);
}

void SO3ControlComponent::position_cmd_callback(
    const quadrotor_msgs::msg::PositionCommand::ConstPtr &cmd)
{
  // 1) 原始期望（EGO 输出）
  Eigen::Vector3d des_pos_raw(
      cmd->position.x,
      cmd->position.y,
      cmd->position.z);

  Eigen::Vector3d des_vel_raw(
      cmd->velocity.x,
      cmd->velocity.y,
      cmd->velocity.z);

  Eigen::Vector3d des_acc_raw(
      cmd->acceleration.x,
      cmd->acceleration.y,
      cmd->acceleration.z);

  double des_yaw_raw     = cmd->yaw;
  double des_yaw_dot_raw = cmd->yaw_dot;

  // 2) 第一次收到 position_cmd 且已经有 odom 时，对齐世界坐标
  if (!g_has_offset && g_has_odom)
  {
    g_pos_offset = des_pos_raw - g_current_pos;
    g_yaw_offset = des_yaw_raw - g_current_yaw;
    g_has_offset = true;

    RCLCPP_INFO(this->get_logger(),
        "SO3Control: init offsets, pos_offset=(%.2f, %.2f, %.2f), yaw_offset=%.2f deg",
        g_pos_offset.x(), g_pos_offset.y(), g_pos_offset.z(),
        g_yaw_offset * 180.0 / M_PI);
  }

  // 3) 应用 offset（如果已经初始化）
  Eigen::Vector3d des_pos_aligned = des_pos_raw;
  double des_yaw_aligned = des_yaw_raw;

  if (g_has_offset)
  {
    des_pos_aligned -= g_pos_offset;
    des_yaw_aligned -= g_yaw_offset;
  }

  // 4) 写回控制器用的期望量（仍然用原来的 des_* 成员）
  des_pos_     = des_pos_aligned;
  des_vel_     = des_vel_raw;
  des_acc_     = des_acc_raw;
  des_yaw_     = des_yaw_aligned;
  des_yaw_dot_ = des_yaw_dot_raw;

  // 保持你之前屏蔽的 kx/kv 逻辑为注释
  // if (cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5) { ... }
  // if (cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5) { ... }

  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}



void SO3ControlComponent::odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom)
{   
    const Eigen::Vector3d position(
        odom->pose.pose.position.x,
        odom->pose.pose.position.y,
        odom->pose.pose.position.z);

    const Eigen::Vector3d velocity(
        odom->twist.twist.linear.x,
        odom->twist.twist.linear.y,
        odom->twist.twist.linear.z);

    // 使用 tf2 提取 yaw
    tf2::Quaternion quat;
    tf2::fromMsg(odom->pose.pose.orientation, quat);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

// === 关键：更新全局当前状态 ===
    g_current_yaw = yaw;
    g_current_pos = position;
    g_has_odom    = true;

    controller_.setPosition(position);
    controller_.setVelocity(velocity);

    if (position_cmd_init_)
    {
        if (!position_cmd_updated_)
            publishSO3Command();
        position_cmd_updated_ = false;
    }
    // ============== 修改开始 ==============
    else 
    {
        // 如果还没收到规划器的指令，就自动锁定在当前位置悬停！
        // 不再依赖 init_z_ 参数，防止参数没设导致坠机
        des_pos_ = position;  // <--- 核心修改：目标位置 = 当前位置
        des_vel_ = Eigen::Vector3d(0, 0, 0);
        des_acc_ = Eigen::Vector3d(0, 0, 0);
        
        // 保持当前的 Yaw，别乱转
        des_yaw_ = current_yaw_; 
        des_yaw_dot_ = 0.0;
        
        publishSO3Command();
    }
    // ============== 修改结束 ==============
}



void SO3ControlComponent::enable_motors_callback(const std_msgs::msg::Bool::ConstPtr &msg)
{
    if (msg->data)
        RCLCPP_INFO(this->get_logger(), "Enabling motors");
    else
        RCLCPP_INFO(this->get_logger(), "Disabling motors");

    enable_motors_ = msg->data;
}

void SO3ControlComponent::corrections_callback(
    const quadrotor_msgs::msg::Corrections::ConstPtr &msg)
{
    corrections_[0] = msg->kf_correction;
    corrections_[1] = msg->angle_corrections[0];
    corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlComponent::imu_callback(const sensor_msgs::msg::Imu &imu)
{
    const Eigen::Vector3d acc(imu.linear_acceleration.x,
                              imu.linear_acceleration.y,
                              imu.linear_acceleration.z);
    controller_.setAcc(acc);
}

void SO3ControlComponent::onInit(void)
{
    // rclcpp::Node::SharedPtr node = this->shared_from_this();
    RCLCPP_INFO(get_logger(), "start SO3ControlComponent");

    declare_parameter("quadrotor_name", "quadrotor");
    declare_parameter("mass", 1.535);

    declare_parameter("use_external_yaw", true);
    // 姿态环 kR
 // === 2. 姿态环 (Inner Loop) - 必须够硬才能执行外环指令 ===
    // 现在的参数 (1.05/1.09) 对于 Iris 来说稍显偏软，导致姿态响应慢
    // 建议适度提高 kR，让飞机姿态锁定更紧
    declare_parameter("gains/rot/x", 1.5); // 原 1.05 -> 提高响应速度
    declare_parameter("gains/rot/y", 1.5); // 原 1.09
    declare_parameter("gains/rot/z", 0.9); // 原 0.90 -> 增强偏航锁定

    // 角速度增益 kΩ 通常配合 kR 调整，保持阻尼比
    // 原来的 0.32 偏低，容易在快速旋转后产生过冲（震荡）
    declare_parameter("gains/ang/x", 0.22); // 原 0.32 -> 稍微降低以减少高频噪声
    declare_parameter("gains/ang/y", 0.22); // 原 0.33
    declare_parameter("gains/ang/z", 0.25); // 原 0.41

    // === 3. 位置环 (Outer Loop) - 解决“偏差大” ===
    // 之前的 3.5 太软了，导致跟踪误差大。这里大幅提升刚度。
    declare_parameter("gains/kx/x", 3.); // 原 3.5 -> 5.5 (强力拉回轨迹)
    declare_parameter("gains/kx/y", 3.5); 
    declare_parameter("gains/kx/z", 5.1); // 原 6.1 -> 8.0 (死锁高度，防掉高)

    // === 4. 速度环 (Damping) - 消除“过冲” ===
    // 配合提升的 Kx，Kv 也要相应跟上，但不要太大以免拖慢系统
    declare_parameter("gains/kv/x", 3.8); // 原 3.6 -> 3.8 (配合 Kx=5.5)
    declare_parameter("gains/kv/y", 3.8);
    declare_parameter("gains/kv/z", 4.5); // 原 6.1 -> 4.5 (Z轴不需要那么大的阻尼)


    declare_parameter("corrections/z", 0.0);
    declare_parameter("corrections/r", 0.0);
    declare_parameter("corrections/p", 0.0);
    declare_parameter("so3_control/init_state_x", 0.0);
    declare_parameter("so3_control/init_state_y", 0.0);
    declare_parameter("so3_control/init_state_z", -10000.0);

    std::string quadrotor_name;
    get_parameter("quadrotor_name", quadrotor_name);
    frame_id_ = "/" + quadrotor_name;

    double mass;
    get_parameter("mass", mass);
    controller_.setMass(mass);

    get_parameter("use_external_yaw", use_external_yaw_);

    get_parameter("gains/rot/x", kR_[0]);
    get_parameter("gains/rot/y", kR_[1]);
    get_parameter("gains/rot/z", kR_[2]);
    get_parameter("gains/ang/x", kOm_[0]);
    get_parameter("gains/ang/y", kOm_[1]);
    get_parameter("gains/ang/z", kOm_[2]);
    get_parameter("gains/kx/x", kx_[0]);
    get_parameter("gains/kx/y", kx_[1]);
    get_parameter("gains/kx/z", kx_[2]);
    get_parameter("gains/kv/x", kv_[0]);
    get_parameter("gains/kv/y", kv_[1]);
    get_parameter("gains/kv/z", kv_[2]);

    get_parameter("corrections/z", corrections_[0]);
    get_parameter("corrections/r", corrections_[1]);
    get_parameter("corrections/p", corrections_[2]);

    get_parameter("so3_control/init_state_x", init_x_);
    get_parameter("so3_control/init_state_y", init_y_);
    get_parameter("so3_control/init_state_z", init_z_);

    rclcpp::SensorDataQoS();
    
    so3_command_pub_ = create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(), std::bind(&SO3ControlComponent::odom_callback, this, std::placeholders::_1));

    position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "position_cmd", rclcpp::SensorDataQoS(), std::bind(&SO3ControlComponent::position_cmd_callback, this, std::placeholders::_1));

    enable_motors_sub_ = create_subscription<std_msgs::msg::Bool>(
        "motors", rclcpp::SensorDataQoS(), std::bind(&SO3ControlComponent::enable_motors_callback, this, std::placeholders::_1));

    corrections_sub_ = create_subscription<quadrotor_msgs::msg::Corrections>(
        "corrections", rclcpp::SensorDataQoS(), std::bind(&SO3ControlComponent::corrections_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&SO3ControlComponent::imu_callback, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SO3ControlComponent)