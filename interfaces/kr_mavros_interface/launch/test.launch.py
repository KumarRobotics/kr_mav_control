from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument("robot", default_value="/", description="Robot namespace")
    odom_arg = DeclareLaunchArgument("odom", default_value="odom", description="Odometry topic")
    so3_cmd_arg = DeclareLaunchArgument("so3_cmd", default_value="so3_cmd", description="SO3 command topic")
    num_props_arg = DeclareLaunchArgument("num_props", default_value="4", description="Number of propellers")
    kf_arg = DeclareLaunchArgument("kf", default_value="2.137145e-6", description="Thrust coefficient")
    lin_cof_a_arg = DeclareLaunchArgument("lin_cof_a", default_value="0.0015", description="Linear coefficient A")
    lin_int_b_arg = DeclareLaunchArgument("lin_int_b", default_value="-1.5334", description="Linear intercept B")

    # Create composable node
    so3_cmd_to_mavros_node = ComposableNode(
        package="kr_mavros_interface",
        plugin="SO3CmdToMavros",
        name="so3cmd_to_mavros",
        namespace=LaunchConfiguration("robot"),
        parameters=[
            {
                "num_props": LaunchConfiguration("num_props"),
                "kf": LaunchConfiguration("kf"),
                "lin_cof_a": LaunchConfiguration("lin_cof_a"),
                "lin_int_b": LaunchConfiguration("lin_int_b"),
                "so3_cmd_timeout": 0.25,
            }
        ],
        remappings=[
            ("~/odom", LaunchConfiguration("odom")),
            ("~/so3_cmd", LaunchConfiguration("so3_cmd")),
            ("~/imu", "mavros/imu/data"),
            ("~/attitude_raw", "mavros/setpoint_raw/attitude"),
            ("~/odom_pose", "mavros/vision_pose/pose"),
        ],
    )

    # Create container
    container = ComposableNodeContainer(
        name="so3cmd_to_mavros_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[so3_cmd_to_mavros_node],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_arg,
            odom_arg,
            so3_cmd_arg,
            num_props_arg,
            kf_arg,
            lin_cof_a_arg,
            lin_int_b_arg,
            container,
        ]
    )
