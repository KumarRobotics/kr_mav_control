#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import tf
import math
import copy
import tf2_ros
import tf2_geometry_msgs

import actionlib

from std_srvs.srv import Empty, Trigger, SetBool, SetBoolRequest
from std_srvs.srv import EmptyResponse

from geometry_msgs.msg import PoseStamped, Pose, Vector3
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from kr_mav_manager.srv import Vec4, Vec4Request

from kr_multi_mav_manager.srv import RawPosFormation, RawPosFormationRequest

from kr_tracker_msgs.msg import LissajousTrackerAction, LissajousTrackerGoal, LissajousTrackerResult
from kr_tracker_msgs.msg import LineTrackerAction, LineTrackerGoal, LineTrackerResult
from kr_tracker_msgs.srv import Transition

def general_service_cb(userdata, response):
  if response.success:
    return 'succeeded'
  else:
    rospy.logerr(response.message)
    return 'aborted'

class GoTo(smach.State):
  def __init__(self, mav_name):
    smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['vec4_goal'])
    self.mav_name = mav_name
    self.client = actionlib.SimpleActionClient('/' + self.mav_name + '/trackers_manager/line_tracker_min_jerk/LineTracker', LineTrackerAction)
    rospy.sleep(0.1)

  def execute(self, userdata):

    goal = LineTrackerGoal();
    goal.x = copy.copy(userdata.vec4_goal[0]);
    goal.y = copy.copy(userdata.vec4_goal[1]);
    goal.z = copy.copy(userdata.vec4_goal[2]);
    goal.yaw = copy.copy(userdata.vec4_goal[3]);
    goal.v_des = 1.0;
    goal.a_des = 0.5;
    goal.relative = False;

    self.client.send_goal(goal)

    try:
      transition_tracker = rospy.ServiceProxy('/' + self.mav_name + '/trackers_manager/transition', Transition)
      resp1 = transition_tracker('kr_trackers/LineTrackerMinJerk')
      print(resp1)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      return 'aborted'

    self.client.wait_for_result()
    return 'succeeded'

class Lissajous(smach.State):
  def __init__(self, mav_name):
    smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
    self.mav_name = mav_name
    self.client = actionlib.SimpleActionClient('/' + self.mav_name + '/trackers_manager/lissajous_tracker/LissajousTracker', LissajousTrackerAction)
    rospy.sleep(0.5)

  def execute(self, userdata):

    goal = LissajousTrackerGoal()
    goal.x_amp =  1.25
    goal.y_amp = 1.25
    goal.z_amp =  0.75
    goal.yaw_amp = 3.1415
    goal.x_num_periods = 12
    goal.y_num_periods = 12
    goal.z_num_periods = 16
    goal.yaw_num_periods = 5
    goal.period = 60
    goal.num_cycles = 1
    goal.ramp_time = 2

    self.client.send_goal(goal)

    try:
      transition_tracker = rospy.ServiceProxy('/' + self.mav_name + '/trackers_manager/transition', Transition)
      resp1 = transition_tracker('kr_trackers/LissajousTracker')
      print(resp1)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      return 'aborted'

    self.client.wait_for_result()

    return 'succeeded'

def Lissajous1(mav1_name):

  lissajous1_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=['mav1_end'])

  with lissajous1_sm:

    smach.StateMachine.add('LISSAJOUS1', Lissajous(mav1_name),
        transitions={'succeeded':'MAV1_END', 'aborted':'aborted'})

    smach.StateMachine.add('MAV1_END', GoTo(mav1_name),
        remapping={'vec4_goal':'mav1_end'},
        transitions={'succeeded':'succeeded', 'aborted':'aborted'})

  return lissajous1_sm

def Lissajous2(mav2_name):

  lissajous2_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=['lissajous_start', 'mav2_end'])

  with lissajous2_sm:

    smach.StateMachine.add('MAV2_START', GoTo(mav2_name),
        remapping={'vec4_goal':'lissajous_start'},
        transitions={'succeeded':'LISSAJOUS2', 'aborted':'aborted'})

    smach.StateMachine.add('LISSAJOUS2', Lissajous(mav2_name),
        transitions={'succeeded':'MAV2_END', 'aborted':'aborted'})

    smach.StateMachine.add('MAV2_END', GoTo(mav2_name),
        remapping={'vec4_goal':'mav2_end'},
        transitions={'succeeded':'succeeded', 'aborted':'aborted'})

  return lissajous2_sm

def main():

  rospy.init_node('demo_lissajous')

  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
  sm.userdata.mav1_name = 'dragonfly1'
  sm.userdata.mav2_name = 'dragonfly2'

  sm.userdata.mav1_start = [3.0, 0.0, 1.5, 0.0]
  sm.userdata.mav2_start = [2.0, 0.0, 1.5, 0.0]

  sm.userdata.mav1_end = [2.0, 1.5, 1.5, 0.0]
  sm.userdata.mav2_end = [2.0, -1.5, 1.5, 0.0]

  # Run the state machine with the introspection server
  sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT/MAIN');
  sis.start();

  with sm:

    smach.StateMachine.add('MAV1_START', GoTo(sm.userdata.mav1_name),
        remapping={'vec4_goal':'mav1_start'},
        transitions={'succeeded':'MAV2_START', 'aborted':'aborted'})

    smach.StateMachine.add('MAV2_START', GoTo(sm.userdata.mav2_name),
        remapping={'vec4_goal':'mav2_start'},
        transitions={'succeeded':'CON', 'aborted':'aborted'})

    # Create the sub SMACH state machine
    sm_con = smach.Concurrence(outcomes=['succeeded', 'aborted'],
                              default_outcome='succeeded',
                              outcome_map={'succeeded':
                                  { 'LISSAJOUS1':'succeeded',
                                    'LISSAJOUS2':'succeeded'}},
                              input_keys=['lissajous_start', 'mav1_end', 'mav2_end'])
    # Open the container
    with sm_con:
        # Add states to the container
        smach.Concurrence.add('LISSAJOUS1', Lissajous1(sm.userdata.mav1_name))
        smach.Concurrence.add('LISSAJOUS2', Lissajous2(sm.userdata.mav2_name))

    smach.StateMachine.add('CON', sm_con,
        remapping={'lissajous_start':'mav1_start', 'mav1_end':'mav1_end', 'mav2_end': 'mav2_end'},
        transitions={'succeeded':'succeeded', 'aborted':'aborted'})

  # Execute SMACH plan
  outcome = sm.execute()

  rospy.spin()

  sis.stop()

if __name__ == '__main__':
  main()
