#! /usr/bin/env python
import math
import copy

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from std_trackers.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal
from trackers_manager.srv import Transition

class WpToAction(object):
  def __init__(self):
    self.ns = '/quadrotor'

    self.client = actionlib.SimpleActionClient('trackers_manager/trajectory_tracker/TrajectoryTracker', TrajectoryTrackerAction)
    rospy.loginfo("Waiting for server")
    self.client.wait_for_server()
    rospy.loginfo("Connected!")

    rospy.Subscriber("/waypoints", Path, self.callback)

  def callback(self, data):

    goal = TrajectoryTrackerGoal()
    for dt in data.poses:
      goal.waypoints.append(copy.deepcopy(dt.pose))

    print goal

    self.client.send_goal(goal)

    rospy.wait_for_service('trackers_manager/transition')
    try:
      transition_tracker = rospy.ServiceProxy('trackers_manager/transition', Transition)
      resp1 = transition_tracker('std_trackers/TrajectoryTracker')
      print resp1
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def main():
  rospy.init_node('wp_to_action')

  wta = WpToAction()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()