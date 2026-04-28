import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # KR interface paths & configurations
    betaflight_config_file = os.path.join(
        get_package_share_directory('kr_betaflight_interface'),
        'config',
        'quadrotor.yaml'
    )

    trackers_manager_config_file = os.path.join(
        get_package_share_directory('quadrotor_interface'),
        'config',
        'quadrotor_trackers.yaml'
    )

    so3_config_file = os.path.join(
        get_package_share_directory('quadrotor_interface'),
        'config',
        'quadrotor_gains.yaml'
    )

    # KR interface arguments
    kr_args = [
        DeclareLaunchArgument('robot', default_value='quadrotor'), # set robot namespace        
        DeclareLaunchArgument('mass', default_value='0.65'), # set mass AUW
        DeclareLaunchArgument('odom', default_value='control_odom'), # set odom topic (vio/ukf/vicon)
        DeclareLaunchArgument('so3_cmd', default_value='so3_cmd'),
        DeclareLaunchArgument('zed_enable', default_value='false'),
        DeclareLaunchArgument('ukf_enable', default_value='false'),
        DeclareLaunchArgument('vicon_enable', default_value='true'),
    ]

    # Define launch arguments
    vicon_args = [
        # Adding mocap vicon specific parameters
        DeclareLaunchArgument('mocap_server', default_value='192.168.8.2'),
        DeclareLaunchArgument('mocap_frame_rate', default_value='100'),
        DeclareLaunchArgument('mocap_max_accel', default_value='10.0'),
    ]    

    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )
    
    # ZED camera arguments
    zed_args = [        
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('camera_model', default_value='zedm'),
        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path)),
        DeclareLaunchArgument('custom_baseline', default_value='0.0'),
        DeclareLaunchArgument('enable_gnss', default_value='false'),
        DeclareLaunchArgument('publish_svo_clock', default_value='false'),
    ]    

    # Initialize launch description with all arguments
    ld = LaunchDescription(vicon_args + kr_args + zed_args)

    # Create a main component container for all composable nodes
    main_container = ComposableNodeContainer(
        name="control_container",
        namespace="",  # Empty namespace for container, individual nodes will have their own namespaces
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="kr_betaflight_interface",
                plugin="SO3CmdToBetaflight",
                name="so3cmd_to_betaflight",
                namespace=LaunchConfiguration('robot'),
                parameters=[betaflight_config_file],
                remappings=[
                    ('so3_cmd', LaunchConfiguration('so3_cmd')),
                    ('odom', LaunchConfiguration('odom')),
                    ('imu', 'zed/zed_node/imu/data'),
                ]
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManager",
                name="trackers_manager",
                namespace=LaunchConfiguration('robot'),
                parameters=[trackers_manager_config_file],
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManagerLifecycleManager",
                name="lifecycle_manager",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    {'node_name': "trackers_manager"}
                ]
            ),
            ComposableNode(
                package="kr_mav_controllers",
                plugin="SO3ControlComponent",
                name="so3_controller",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    [so3_config_file],
                    {'quadrotor_name': LaunchConfiguration('robot')},
                ],
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('zed_enable')),
                package="zed_components",
                plugin="stereolabs::ZedCamera",
                name="zed_node",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/', LaunchConfiguration('camera_model'), '.yaml'],
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/common_stereo.yaml'],
                    # Finally apply launch-specific overrides
                    {
                        'general.camera_name': LaunchConfiguration('camera_name'),
                        'general.camera_model': LaunchConfiguration('camera_model'),
                        'pos_tracking.publish_tf': LaunchConfiguration('publish_tf'),
                        'pos_tracking.publish_map_tf': LaunchConfiguration('publish_map_tf'), 
                        'sensors.publish_imu_tf': LaunchConfiguration('publish_imu_tf', default='true'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('ukf_enable')),
                package="quadrotor_ukf_ros2",
                plugin="QuadrotorUKFNode",
                name="quadrotor_ukf_ros2",
                namespace=LaunchConfiguration('robot'),
                parameters=[{
                    'odom': LaunchConfiguration('odom'),
                    'imu_frame_id': 'zed_imu_link',
                    'imu_rotated_frame_id': 'zed_camera_link',                    
                    'base_link': 'zed_camera_link'
                }],
                remappings=[
                    ('odom', 'zed_node/odom'),
                    ('imu', 'zed_node/imu/data'),
                ],
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('vicon_enable')),
                package='mocap_vicon',
                plugin='mocap::ViconDriverComponent',
                name='vicon',
                namespace='vicon',
                parameters=[
                    {'server_address': LaunchConfiguration('mocap_server')},
                    {'frame_rate': LaunchConfiguration('mocap_frame_rate')},
                    {'max_accel': LaunchConfiguration('mocap_max_accel')},
                    {'publish_tf': False},
                    {'publish_pts': False},
                    {'fixed_frame_id': 'world'},
                    {'model_list': ['']},
                ],
                remappings=[
                    (['/vicon/', LaunchConfiguration('robot'), '/odom'],
                     ['/', LaunchConfiguration('robot'), '/control_odom']),
                ]
            ),
        ],
        output='screen',
    )
    
    ld.add_action(main_container)
    
    # robot state publisher to publish URDF and static transforms for zed
    ld.add_action(
        Node(
            condition=IfCondition(LaunchConfiguration('zed_enable')),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=[LaunchConfiguration('camera_name'), '_state_publisher'],
            namespace=LaunchConfiguration('robot'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('publish_svo_clock'),
                'robot_description': Command([
                    'xacro', ' ', LaunchConfiguration('xacro_path'),
                    ' camera_name:=', LaunchConfiguration('camera_name'),
                    ' camera_model:=', LaunchConfiguration('camera_model'),
                    ' custom_baseline:=', LaunchConfiguration('custom_baseline')
                ])
            }]
        )
    )

    ld.add_action(
        Node(
            package="kr_mav_manager",
            executable="mav_services",
            namespace=LaunchConfiguration('robot'),
            name="mav_services",
            output='screen',
            parameters = [
                {'mass': LaunchConfiguration('mass')},
                {'robot': LaunchConfiguration('robot')},
            ],
        )
    )

    return ld
