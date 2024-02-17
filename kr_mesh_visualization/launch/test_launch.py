from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

  input_topic_launch_arg = DeclareLaunchArgument(
    "~/input",
    default_value=TextSubstitution(text="/quadrotor/odom")
  )
  
  new_frame_id_launch_arg = DeclareLaunchArgument(
    "new_frame_id", default_value=TextSubstitution(text="")
  )

  msg_type_launch_arg = DeclareLaunchArgument(
    "msg_type",
    default_value=TextSubstitution(text="nav_msgs/msg/Odometry")
  )

  mesh_visualization_node = Node(
    package="kr_mesh_visualization",
    executable="kr_mesh_visualization",
    name="mesh_visualization",
    output="screen",
    remappings=[
      ("~/input", LaunchConfiguration("~/input"))
    ],
    parameters=[
      {"mesh_resource": "package://kr_mesh_visualization/mesh/hummingbird.mesh"},
      {"color/r": 0.0},
      {"color/g": 0.0},
      {"color/b": 1.0},
      {"color/a": 0.7},
      {"new_frame_id": LaunchConfiguration("new_frame_id")},
      {"msg_type": LaunchConfiguration("msg_type")}
    ]
  )

  return LaunchDescription([
    input_topic_launch_arg,
    new_frame_id_launch_arg,
    msg_type_launch_arg,
    mesh_visualization_node
  ])
