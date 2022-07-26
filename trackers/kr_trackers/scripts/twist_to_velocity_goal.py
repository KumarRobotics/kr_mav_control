#! /usr/bin/env python3
from __future__ import print_function

import rospy
import actionlib

from geometry_msgs.msg import Twist
#from kr_tracker_msgs.msg import VelocityTrackerAction, VelocityTrackerGoal
from kr_tracker_msgs.msg import VelocityGoal, TrackerStatus
from kr_tracker_msgs.srv import Transition

class TwistToAction(object):
  def __init__(self):
    self.ns = '/quadrotor'

    # self.client = actionlib.SimpleActionClient('trackers_manager/velocity_tracker/VelocityTrackerAction', VelocityTrackerAction)
    # rospy.loginfo("Waiting for server")
    # self.client.wait_for_server()
    # rospy.loginfo("Connected!")
    self.current_tracker = ""
    self.vel_pub = rospy.Publisher("trackers_manager/velocity_tracker/goal", VelocityGoal, queue_size=1)
    rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=1)
    rospy.Subscriber("trackers_manager/status", TrackerStatus, self.status_callback, queue_size=1)

  def status_callback(self, data):
    self.current_tracker = data.tracker

  def callback(self, data):

    goal = VelocityGoal()
    goal.header.stamp = rospy.Time.now()
    goal.vx = data.linear.x
    goal.vy = data.linear.y
    goal.vz = data.linear.z
    goal.vyaw = data.angular.z
    goal.use_position_gains = True
    print(goal)

    self.vel_pub.publish(goal)

    # self.client.send_goal(goal)

    if(self.current_tracker != "kr_trackers/VelocityTracker"):
      rospy.wait_for_service('trackers_manager/transition')
      try:
        transition_tracker = rospy.ServiceProxy('trackers_manager/transition', Transition)
        resp1 = transition_tracker('kr_trackers/VelocityTracker')
        print(resp1)
      except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
  rospy.init_node('twist_to_velocity_goal')

  tta = TwistToAction()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()
