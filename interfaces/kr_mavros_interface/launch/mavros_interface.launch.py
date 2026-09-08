import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Airframe properties (thrust curve, timeouts, vision_pose rate) live in the
    # config file rather than being duplicated as launch arguments here and in
    # neurofly_interface/launch/system_launch.launch.py. Override the file with
    # config_file:= to fly a different airframe.
    default_config_file = os.path.join(
        get_package_share_directory("kr_mavros_interface"),
        "config",
        "neurofly.yaml",
    )

    # Declare launch arguments
    robot_arg = DeclareLaunchArgument("robot", default_value="/", description="Robot namespace")
    odom_arg = DeclareLaunchArgument("odom", default_value="odom", description="Odometry topic")
    so3_cmd_arg = DeclareLaunchArgument("so3_cmd", default_value="so3_cmd", description="SO3 command topic")
    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=default_config_file, description="SO3CmdToMavros parameter file"
    )

    # Create composable node
    so3_cmd_to_mavros_node = ComposableNode(
        package="kr_mavros_interface",
        plugin="SO3CmdToMavros",
        name="so3cmd_to_mavros",
        namespace=LaunchConfiguration("robot"),
        parameters=[LaunchConfiguration("config_file")],
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
            config_file_arg,
            container,
        ]
    )
