kr_mesh_visualization
=============

Package to visualize the drone in RViz.

## Message types supported:

Three message types are supported in this package in order to subscribe to the pose data and visualize the drone. The default message type that is being subscribed to is `nav_msgs/msg/Odometry`. The other message types supported are `geometry_msgs/msg/PoseStamped` and `geometry_msgs/msg/PoseWithCovarianceStamped`. The subscribers for the other two message types have been commented out since ROS2 Humble does not allow multiple subscribers to subscribe to a topic with different message types. 

If you need to use another a particular message type, please uncomment the respective subscriber and comment out the rest.

## Testing kr_mesh_visualization

To run a demo of the package: 

1. You can either run package directly using `ros2 run` or using the launch file.

    a. First method: use the following command
    ```
    ros2 run kr_mesh_visualization kr_mesh_visualization
    ```

    b: Second method: use the following command
    ```
    ros2 launch kr_mesh_visualization test_launch.py
    ```

2. Launch RViz using the command `rviz2`. Add the `Marker` topic to `Displays`.

3. Publish a demo message to see the mesh visualization. The command is

    ```
    ros2 topic pub /topic_name nav_msgs/msg/Odometry '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, child_frame_id: "", pose: {pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
    ```

    Based on the method used to run the package, the `/topic_name` might change.

    a. For first method `/topic_name` is `/mesh_visualization/input`

    b. For the second method `/topic_name` is `/quadrotor/odom`