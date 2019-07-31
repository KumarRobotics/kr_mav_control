#!/bin/bash
if [ $# -eq 0 ]; then
  echo "Input number(integer) of MAVs as first argument"
  exit 1
fi

NUM_MAV=$1

if echo $NUM_MAV | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Running simulated $NUM_MAV MAVs"
else
  echo "Input number(integer) of MAVs as first argument"
  exit 1
fi

if [ ${NUM_MAV} -eq 1 ]; then
  RQT_GUI=rqt_mav_manager
else
  RQT_GUI=rqt_multi_mav_gui
fi

# TODO parse this from command line? Possibly list of mav ids and namespace?
MAV_NAMESPACE=dragonfly

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=demo_gs${NUM_MAV}

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

ODOM_TOPIC=ground_truth/odom

# Generate rviz config file for specific mav from default one
RVIZ_CONFIG_FILE="$HOME/.ros/wp_nav.rviz"
LAUNCH_PATH=$(rospack find quadrotor_simulator)
cp $LAUNCH_PATH/launch/wp_nav.rviz ~/.ros/wp_nav.rviz
sed -i "s/quadrotor/temp/g" ~/.ros/wp_nav.rviz

# Generate multi_mav_manger yaml config file based on number of robots
cp $(rospack find multi_mav_manager)/config/dragonfly/multi_mav_manager_single.yaml ~/.ros/multi_mav_manager.yaml
for id in $(seq 1 $NUM_MAV)
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  sed -i "1a\  '"${MAV_NAME}"'," ~/.ros/multi_mav_manager.yaml
  echo "/${MAV_NAME}/active: true" >> ~/.ros/multi_mav_manager.yaml
done

# Make mouse useful in copy mode
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Main"
#tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
#tmux split-window -t $SESSION_NAME
#tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; export DISPLAY=:0; rosrun rviz rviz -d ${RVIZ_CONFIG_FILE}" Enter
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roslaunch mrsl_quadrotor_launch gazebo.launch world:=empty" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; export DISPLAY=:0; rqt --standalone ${RQT_GUI}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch multi_mav_manager multi_mav_manager.launch odom_topic:=${ODOM_TOPIC} config_path:=$HOME/.ros/" Enter
tmux select-layout -t $SESSION_NAME tiled

# Launch each mav in a new tmux window
for id in $(seq 1 $NUM_MAV)
do

  # Append rviz/Marker for cuurent mav id.
  MAV_NAME=${MAV_NAMESPACE}${id}
  sed -i "108a\    - Class: rviz/Marker\n      Enabled: true\n      Marker Topic: /quadrotor/mesh_visualization/robot\n      Name: quadrotor\n      Namespaces:\n        /quadrotor/mesh_visualization: true\n      Queue Size: 100\n      Value: true" ~/.ros/wp_nav.rviz
  sed -i "s/quadrotor/${MAV_NAME}/g" ~/.ros/wp_nav.rviz

  tmux new-window -t $SESSION_NAME -n "r${id}"

  # TODO generate poses on circle instead. Separated by robot size
  # Generate random poses x, y
  POS_X=$(( $RANDOM % 10 ))
  POS_Y=$(( $RANDOM % 10 ))

  # Generate random colors [0-1]
  v=$[100 + (RANDOM % 100)]$[1000 + (RANDOM % 1000)]
  COL_R=0.${v:1:2}${v:4:3}
  v=$[100 + (RANDOM % 100)]$[1000 + (RANDOM % 1000)]
  COL_G=0.${v:1:2}${v:4:3}
  v=$[100 + (RANDOM % 100)]$[1000 + (RANDOM % 1000)]
  COL_B=0.${v:1:2}${v:4:3}

  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch mrsl_quadrotor_launch spawn.launch robot_type:=pelican robot:=${MAV_NAME} x:=${POS_X} y:=${POS_Y}" Enter
  tmux split-window -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch mav_manager example_control.launch model:=${MAV_NAME} odom_topic:=${ODOM_TOPIC} mass:=0.5" Enter
  tmux split-window -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; rosrun trackers_manager waypoints_to_action.py __ns:=${MAV_NAME}" Enter
  tmux select-layout -t $SESSION_NAME even-horizontal
  tmux split-window -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; rosrun trackers_manager twist_to_action.py __ns:=${MAV_NAME}" Enter
  tmux split-window -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; cd $(rospack find mav_manager)/scripts/; ./takeoff.sh ${MAV_NAME}"
  tmux select-layout -t $SESSION_NAME even-horizontal
done

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME