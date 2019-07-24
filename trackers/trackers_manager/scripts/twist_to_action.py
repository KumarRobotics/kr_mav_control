#! /usr/bin/env python
import math
import copy

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_trackers.msg import VelocityTrackerAction, VelocityTrackerGoal
from trackers_manager.srv import Transition

class WpToAction(object):
  def __init__(self):
    self.ns = '/quadrotor'

    self.client = actionlib.SimpleActionClient(self.ns+'/trackers_manager/velocity_tracker/VelocityTrackerAction', VelocityTrackerAction)
    rospy.loginfo("Waiting for server")
    self.client.wait_for_server()
    rospy.loginfo("Connected!")

    rospy.Subscriber(self.ns+"/cmd_vel", Twist, self.callback)

  def callback(self, data):

    goal = VelocityTrackerGoal()
    goal.vx = copy.deepcopy(data.linear.x);
    goal.vy = copy.deepcopy(data.linear.y);
    goal.vz = copy.deepcopy(data.linear.z);
    goal.vyaw = copy.deepcopy(data.angular.z);

    print goal

    self.client.send_goal(goal)

    rospy.wait_for_service(self.ns+'/trackers_manager/transition')
    try:
      transition_tracker = rospy.ServiceProxy(self.ns+'/trackers_manager/transition', Transition)
      resp1 = transition_tracker('std_trackers/VelocityTrackerAction')
      print resp1
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def main():
  rospy.init_node('twist_to_action')

  wta = WpToAction()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()