import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    # map
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 50.0},
            {'map/y_size': 40.0},
            {'map/z_size': 5.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 30},
            {'ObstacleShape/lower_rad': 0.4}, # 原 0.3
            {'ObstacleShape/upper_rad': 0.8}, # 原 0.6
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 30},
            {'ObstacleShape/radius_l': 1.5},
            {'ObstacleShape/radius_h': 1.8},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 1.5},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 3.0}
        ],
        condition = UnlessCondition(use_mockamap)
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(0.5),    
            'max_acc': str(1.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),  # 必须是 2 才能跑序列点
            'point_num': str(4),    # 这里设置总共有 8 个点
            
# --- 第1阶段：东向直行建立速度 (Heading 0°) ---
            # 矢量: (-5 -> 5, -4) 纯东方向
            'point0_x': str(5.0),
            'point0_y': str(-4.0),
            'point0_z': str(1.5),
            
            # --- 第2阶段：东北切角 (Heading 45°) ---
            # 矢量: (3, 3) 变化45度
            'point1_x': str(15.0),
            'point1_y': str(-1.0),
            'point1_z': str(1.5),
            
            # --- 第3阶段：正北爬升 (Heading 90°) ---
            # 矢量: (0, 2) 变化45度
            'point2_x': str(15.0),
            'point2_y': str(-10.0),
            'point2_z': str(1.5), # 爬升至 2.0m
            
            # 矢量: (-3, 3) 变化45度
            'point3_x': str(0.0),
            'point3_y': str(-10.0),
            'point3_z': str(1.8),
            
            
        }.items()
    )
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # Include simulator 
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(0.0),
            'init_y_': str(0.0),
            'init_z_': str(1.5),
            'odometry_topic': odom_topic
        }.items()
    )
    
# ==========================================
    # [补丁] 恢复感知能力 (修正版)
    # ==========================================
    pcl_render_node = Node(
        package='local_sensing', 
        executable='pcl_render_node',
        name=['drone_', drone_id, '_pcl_render_node'],
        output='screen',
        parameters=[
            {'sensing_horizon': 3.0},
            {'sensing_rate': 7.0},
            {'estimation_rate': 7.0},
            {'map/x_size': map_size_x},
            {'map/y_size': map_size_y},
            {'map/z_size': map_size_z},
            {'grid_map/resolution': 0.1},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.4},
            {'camera/cx': 321.04638671875},
            {'camera/cy': 243.44969177246094},
            {'camera/fx': 387.229248046875},
            {'camera/fy': 387.229248046875},
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', odom_topic),
            
            # 这里的 cloud 可以不接了，或者留着给 RViz 看
            ('pcl_render_node/cloud', ['drone_', drone_id, '_pcl_render_node/cloud']),
            
            # === 🔴 关键：把深度图接通 ===
            # 必须加上前缀，因为 advanced_param 默认就在听 drone_0_pcl_render_node/depth
            ('depth', ['drone_', drone_id, '_pcl_render_node/depth']) 
        ]
    )
    tf_body_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 意思：相机装在机身中心
        arguments=['0.0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 别忘了加进去


    ld = LaunchDescription()
    ld.add_action(pcl_render_node)   
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(tf_body_camera)
    # 添加 Map Generator 节点
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    # ld.add_action(simulator_include)

    return ld