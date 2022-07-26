#! /usr/bin/env python3
from __future__ import print_function

import rospy
import actionlib

from nav_msgs.msg import Path
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal
from kr_tracker_msgs.srv import Transition

class WpToAction(object):
  def __init__(self):
    self.client = actionlib.SimpleActionClient('trackers_manager/trajectory_tracker/TrajectoryTracker', TrajectoryTrackerAction)
    rospy.loginfo("Waiting for server")
    self.client.wait_for_server()
    rospy.loginfo("Connected!")

    rospy.Subscriber("waypoints", Path, self.callback, queue_size=1)

  def callback(self, data):

    goal = TrajectoryTrackerGoal()
    for dt in data.poses:
      goal.waypoints.append(dt.pose)

    print(goal)

    self.client.send_goal(goal)

    rospy.wait_for_service('trackers_manager/transition')
    try:
      transition_tracker = rospy.ServiceProxy('trackers_manager/transition', Transition)
      resp = transition_tracker('kr_trackers/TrajectoryTracker')
      print(resp)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)

def main():
  rospy.init_node('wp_to_action')

  wta = WpToAction()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()
