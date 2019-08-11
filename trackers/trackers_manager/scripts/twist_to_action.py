#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib

from geometry_msgs.msg import Twist
from tracker_msgs.msg import VelocityTrackerAction, VelocityTrackerGoal
from tracker_msgs.srv import Transition

class TwistToAction(object):
  def __init__(self):
    self.ns = '/quadrotor'

    self.client = actionlib.SimpleActionClient('trackers_manager/velocity_tracker/VelocityTrackerAction', VelocityTrackerAction)
    rospy.loginfo("Waiting for server")
    self.client.wait_for_server()
    rospy.loginfo("Connected!")

    rospy.Subscriber("cmd_vel", Twist, self.callback)

  def callback(self, data):

    goal = VelocityTrackerGoal()
    goal.vx = data.linear.x
    goal.vy = data.linear.y
    goal.vz = data.linear.z
    goal.vyaw = data.angular.z

    print(goal)

    self.client.send_goal(goal)

    rospy.wait_for_service('trackers_manager/transition')
    try:
      transition_tracker = rospy.ServiceProxy('trackers_manager/transition', Transition)
      resp1 = transition_tracker('std_trackers/VelocityTrackerAction')
      print(resp1)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)

def main():
  rospy.init_node('twist_to_action')

  tta = TwistToAction()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()
