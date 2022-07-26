#! /usr/bin/env python3
from __future__ import print_function

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal, LineTrackerAction, LineTrackerGoal

from kr_tracker_msgs.srv import Transition
from std_srvs.srv import Trigger, SetBool
from kr_mav_manager.srv import Vec4

class KrMavInterface(object):

  def __init__(self, mav_namespace='dragonfly', id=0):
    self.mav_namespace = mav_namespace
    self.id = id

    self.mav_name = self.mav_namespace + str(self.id)

    self.pub_vel = rospy.Publisher('/' + self.mav_name + '/cmd_vel', Twist, queue_size=10)

    self.odom = Odometry()
    self.sub_odom = rospy.Subscriber('/' + self.mav_name + '/odom', Odometry, self.update_odom)

    self.line_tracker_client = actionlib.SimpleActionClient(self.mav_name +
        '/trackers_manager/line_tracker_min_jerk/LineTracker', LineTrackerAction)
    self.traj_tracker_client = actionlib.SimpleActionClient(self.mav_name +
        '/trackers_manager/trajectory_tracker/TrajectoryTracker', TrajectoryTrackerAction)
    self.circle_tracker_client = actionlib.SimpleActionClient(self.mav_name +
        '/trackers_manager/circle_tracker/CircleTracker', CircleTrackerAction)
    self.traj_tracker_status = ""

  def update_odom(self, msg):
    self.odom = msg

  def get_odom(self):
    return self.odom

  def motors_on(self):
    try:
      motors = rospy.ServiceProxy('/' + self.mav_name + '/mav_services/motors', SetBool)
      resp = motors(True)
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s"%e)
      return 'aborted'

  def motors_off(self):
    try:
      motors = rospy.ServiceProxy('/' + self.mav_name + '/mav_services/motors', SetBool)
      resp = motors(False)
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s"%e)
      return 'aborted'

  def take_off(self):
    try:
      takeoff = rospy.ServiceProxy('/' + self.mav_name + '/mav_services/takeoff', Trigger)
      resp = takeoff()
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s"%e)
      return 'aborted'

  def hover(self):
    rospy.logwarn("Transition to hover")
    self.traj_tracker_client.cancel_all_goals()
    rospy.wait_for_service('/' + self.mav_name  + '/mav_services/hover')
    try:
      srv = rospy.ServiceProxy('/' + self.mav_name + '/mav_services/hover', Trigger)
      resp = srv()
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      return False

  def land(self):
    rospy.logwarn("Transition to land")
    self.traj_tracker_client.cancel_all_goals()
    rospy.wait_for_service('/' + self.mav_name + '/mav_services/land')
    try:
      srv = rospy.ServiceProxy('/' + self.mav_name + '/mav_services/land', Trigger)
      resp = srv()
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      return False

  def set_vel(self, vx=0.0, vy=0.0, vz=0.0, ax=0.0, ay=0.0, az=0.0):
    command = Twist()
    command.linear.x = vx
    command.linear.y = vy
    command.linear.z = vz
    command.angular.x = ax
    command.angular.y = ay
    command.angular.z = az

    self.pub_vel.publish(command)

  def send_wp(self, x, y, z, yaw):
    try:
      rospy.wait_for_service('/' + self.mav_name + '/mav_services/goTo')
      srv = rospy.ServiceProxy('/' + self.mav_name +'/mav_services/goTo', Vec4)
      resp = srv([x, y, z, yaw])
      rospy.loginfo(resp)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      return False

  def send_wp_block(self, x, y, z, yaw, vel=1.0, acc=0.5, relative=False):
    goal = LineTrackerGoal();
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal.yaw = yaw;
    goal.v_des = 1.0;
    goal.a_des = 0.5;
    goal.relative = False;

    self.line_tracker_client.send_goal(goal)
    self.transition_service_call('LineTrackerMinJerk')
    rospy.logwarn("Waiting for Line Tracker to complete")
    self.line_tracker_client.wait_for_result()

  def transition_service_call(self, tracker_name):
    rospy.loginfo('waiting for transition service for ' + tracker_name)
    rospy.wait_for_service('/' + self.mav_name +'/trackers_manager/transition')
    try:
      tt = rospy.ServiceProxy('/' + self.mav_name +'/trackers_manager/transition', Transition)
      resp = tt('kr_trackers/' + tracker_name)
      rospy.loginfo(resp)
      if resp.success == False or "already active" in resp.message:
        return False
      return True
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      return False
