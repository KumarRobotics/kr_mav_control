kr_mesh_visualization
=============

Package to visualize the drone in RViz.

## Message types supported:

Three message types are supported in this package in order to subscribe to the pose data and visualize the drone. The default message type that is being subscribed to is `nav_msgs/msg/Odometry`. The other message types supported are `geometry_msgs/msg/PoseStamped` and `geometry_msgs/msg/PoseWithCovarianceStamped`. ROS2 does not support multiple subscribers to the same topic with different message type.

In order to change the message type for the subscriber, you can use the parameter `msg_type` to change the message type to any one of the 3 supported types. This parameter accepts a string of the message types listed above. 
