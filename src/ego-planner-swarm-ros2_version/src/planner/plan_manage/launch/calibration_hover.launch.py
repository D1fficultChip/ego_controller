import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # =========================================
    # 1. 核心配置参数 (只保留标定必须的)
    # =========================================
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    # 这里的 odom_topic 必须和你 Bridge 输出的一致
    odom_topic = LaunchConfiguration('odom_topic', default='/odom_world') 
    
    # 强制设定为 Manual 模式 (1)，不要改成 2
    flight_type = '1' 
    
    # 初始高度设为 2.5 (配合 PX4 commander takeoff)
    init_z_val = '2.5' 

    # 声明参数
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # =========================================
    # 2. 地图生成器 (跑一个空图，防止 Planner 等不到数据报错)
    # =========================================
    # 我们生成一个极简的地图，障碍物数量为 0
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 20.0},
            {'map/y_size': 20.0},
            {'map/z_size': 5.0},
            {'map/resolution': 0.1},
            {'map/obs_num': 0},        # <--- 关键：0个障碍物
            {'map/circle_num': 0},     # <--- 关键：0个圆环
            {'min_distance': 10.0},    # 安全距离拉满
            {'pub_rate': 1.0}
        ]
    )

    # =========================================
    # 3. 引入 EGO 核心参数 (Advanced Param)
    # =========================================
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': '40.0',
            'map_size_y_': '40.0',
            'map_size_z_': '5.0',
            'odometry_topic': odom_topic,
            'obj_num_set': '0',  # 告诉 Planner 也没障碍物
            
            # 相机参数 (Iris 默认)
            'cx': '321.04638671875',
            'cy': '243.44969177246094',
            'fx': '387.229248046875',
            'fy': '387.229248046875',
            
            # 动力学参数 (标定时由于只是悬停，这些不起作用，但要设个安全值)
            'max_vel': '1.0',
            'max_acc': '1.0',
            'planning_horizon': '7.5',
            
            # 关键设定
            'flight_type': flight_type, 
            'use_distinctive_trajs': 'False', # 简单模式
            
            # 这里的 init_z 没什么实际物理作用(因为我们用真实odom)，但能防止逻辑报错
            'point0_x': '0.0', 'point0_y': '0.0', 'point0_z': init_z_val,
            'point1_x': '0.0', 'point1_y': '0.0', 'point1_z': init_z_val,
            'point2_x': '0.0', 'point2_y': '0.0', 'point2_z': init_z_val,
            'point3_x': '0.0', 'point3_y': '0.0', 'point3_z': init_z_val,
            'point4_x': '0.0', 'point4_y': '0.0', 'point4_z': init_z_val,
            'point_num': '1' 
        }.items()
    )
    
    # =========================================
    # 4. Trajectory Server (轨迹执行器)
    # =========================================
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            # 确保这个话题对齐，方便手动发 goal
            ('/move_base_simple/goal', '/move_base_simple/goal') 
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # =========================================
    # 5. 组合 Launch Description
    # =========================================
    ld = LaunchDescription()
    
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    
    # 注意：这里 **没有** 添加 simulator_include，彻底防止 Odom 冲突
    ld.add_action(map_generator_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)

    return ld