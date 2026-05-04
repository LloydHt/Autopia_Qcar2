import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # =========================================================
    # 1. CHEMINS
    # =========================================================
    pkg_qcar2            = '/home/lloyd/qcar2_ws/install/qcar2/share/qcar2'
    pkg_qcar2_perception = get_package_share_directory('qcar2_perception')
    pkg_ros_gz_sim       = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.expanduser('~/qcar2_ws/src/autopia_gazebo/worlds/autopia.world')
    models_path = os.path.expanduser('~/qcar2_ws/src/autopia_gazebo/models')
    urdf_file   = os.path.join(pkg_qcar2, 'urdf', 'QCar2.urdf')
    rviz_config = os.path.join(pkg_qcar2, 'rviz', 'autopia.rviz')
    ekf_config  = os.path.join(pkg_qcar2_perception, 'config', 'ekf.yaml')

    # =========================================================
    # 2. VARIABLE D'ENVIRONNEMENT — Modèles Gazebo
    # =========================================================
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # =========================================================
    # 3. LECTURE URDF
    # =========================================================
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_desc_rviz   = robot_desc
    robot_desc_gazebo = robot_desc.replace(
        'package://qcar2/',
        'file://' + pkg_qcar2 + '/'
    )

    # =========================================================
    # 4. ROBOT STATE PUBLISHER — Pour RViz (package://)
    # =========================================================
    rsp_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_rviz,
            'frame_prefix': 'qcar2/',
            'use_sim_time': True
        }]
    )

    # =========================================================
    # 5. ROBOT STATE PUBLISHER — Pour Gazebo (file://)
    # =========================================================
    rsp_gazebo_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_gz',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_gazebo,
            'frame_prefix': 'qcar2/',
            'use_sim_time': True
        }],
        remappings=[
            ('robot_description', 'robot_description_gz')
        ]
    )

    # =========================================================
    # 6. GAZEBO
    # =========================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'"{world_path}" -r -v 4'
        }.items()
    )

    # =========================================================
    # 7. SPAWN QCAR2
    # =========================================================
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_qcar2',
        arguments=[
            '-topic', 'robot_description_gz',
            '-name',  'qcar2',
            '-x',     '0.3',
            '-y',     '-8.0',
            '-z',     '0.1',
            '-Y',     '1.57'
        ],
        output='screen'
    )

    # =========================================================
    # 8. BRIDGE ROS <-> GAZEBO
    # =========================================================
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/qcar2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/qcar2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/qcar2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # ★ NOUVEAU — Ground truth Gazebo
            '/world/autopia_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/qcar2/odometry', '/odom'),
            ('/model/qcar2/tf',       '/tf'),
        ],
        output='screen'
    )

    # =========================================================
    # 9. EKF — Filtre de Kalman (odom + IMU)
    # =========================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ]
    )

    # =========================================================
    # 10. GROUND TRUTH PUBLISHER ★ NOUVEAU
    #     Publie /ground_truth depuis Gazebo dynamic_pose
    #     TimerAction : attend 3s que le bridge soit prêt
    # =========================================================
    ground_truth_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='qcar2_perception',
                executable='ground_truth_publisher',
                name='ground_truth_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # =========================================================
    # 11. LIDAR SAFETY NODE
    # =========================================================
    lidar_safety_node = Node(
        package='qcar2_perception',
        executable='lidar_safety_node',
        name='lidar_safety',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =========================================================
    # 12. RVIZ2
    # =========================================================
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # =========================================================
    # 13. RETOUR
    # =========================================================
    return LaunchDescription([
        set_model_path,       # 1. Variables d'environnement
        rsp_rviz_node,        # 2. RSP RViz
        rsp_gazebo_node,      # 3. RSP Gazebo
        gazebo_launch,        # 4. Gazebo
        spawn_node,           # 5. Spawn QCar2
        bridge_node,          # 6. Bridge ROS<->Gazebo
        ekf_node,             # 7. EKF
        lidar_safety_node,    # 8. LiDAR safety
        ground_truth_node,    # 9. Ground truth (après 3s) ★
        #rviz_node,            # 10. RViz (après 3s)
    ])