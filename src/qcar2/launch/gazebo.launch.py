import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURATION DES CHEMINS ---
    pkg_qcar2 = get_package_share_directory('qcar2')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Chemin vers votre monde Autopia
    world_path = os.path.expanduser('~/qcar2_ws/src/autopia_gazebo/worlds/autopia.world')
    
    # Chemin vers vos modèles (MAISONS, ARBRES, SOL)
    # C'est cette ligne qui corrige l'Error Code 14
    models_path = os.path.expanduser('~/qcar2_ws/src/autopia_gazebo/models')
    set_model_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)

    # --- 2. DESCRIPTION DU ROBOT (URDF) ---
    urdf_file = os.path.join(pkg_qcar2, 'urdf', 'QCar2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Correction des chemins des meshes pour Gazebo
    absolute_mesh_path = 'file://' + pkg_qcar2 + '/'
    robot_desc = robot_desc.replace('package://qcar2/', absolute_mesh_path)

    # Nœud Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': 'qcar2/',
            'use_sim_time': True
        }]
    )

    # --- 3. LANCEMENT DE GAZEBO ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'"{world_path}" -r -v 4'}.items() 
    )

    # --- 4. APPARITION DU QCAR 2 ---
    # Positionné sur l'avenue Sud (y=-8), orienté vers le Nord (Y=1.57)
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'qcar2',
            '-x', '0.3',
            '-y', '-8.0',
            '-z', '0.1',
            '-Y', '1.57'
        ],
        output='screen'
    )

    # --- 5. LE PONT (BRIDGE) ROS <-> GAZEBO ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # --- CORRECTION ODOM & IMU ---
            # On utilise le vrai nom Gazebo : /model/qcar2/odometry
            '/model/qcar2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/qcar2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/qcar2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            # On simplifie les noms pour que l'EKF les trouve sans effort
            ('/model/qcar2/odometry', '/odom'),
            ('/model/qcar2/tf', '/tf'),
        ],
        output='screen'
    )
    # --- 6. RETOUR DE LA DESCRIPTION ---
    return LaunchDescription([
        set_model_path, # On définit le chemin des modèles en premier
        rsp_node,
        gazebo_launch,
        spawn_node,
        bridge_node
    ])