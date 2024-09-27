#ifndef TESTER_UTILS_HPP
#define TESTER_UTILS_HPP

#include <actionlib/client/simple_action_client.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/Transition.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <vector>

class TrackersManagerTester
{
 public:
  TrackersManagerTester();
  bool initial_checks();
  void position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &msg);
  void tracker_status_callback(const kr_tracker_msgs::TrackerStatus::ConstPtr &msg);
  void send_transition_request(std::string tracker_name);
  // void done_callback(const actionlib::SimpleClientGoalState &state, const
  // kr_tracker_msgs::LineTrackerActionResultConstPtr &result);
  std::mutex mutex;
  bool srv_response;
  std::string srv_msg;
  bool srv_succeed;

 private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber position_cmd_sub_, tracker_status_sub_;
  ros::ServiceClient transition_client_;

  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> ClientType;
  std::shared_ptr<ClientType> distance_client_;
  std::shared_ptr<ClientType> min_jerk_client_;
};

TrackersManagerTester::TrackersManagerTester() : nh_("")
{
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("trackers_manager/odom", 5, true);
  position_cmd_sub_ = nh_.subscribe<kr_mav_msgs::PositionCommand>("trackers_manager/cmd", 5,
                                                                  &TrackersManagerTester::position_cmd_callback, this);
  tracker_status_sub_ = nh_.subscribe<kr_tracker_msgs::TrackerStatus>(
      "trackers_manager/status", 5, &TrackersManagerTester::tracker_status_callback, this);
  distance_client_ = std::make_shared<ClientType>("trackers_manager/line_tracker_distance/LineTracker", true);
  min_jerk_client_ = std::make_shared<ClientType>("trackers_manager/line_tracker_min_jerk/LineTracker", true);
  transition_client_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");
}

void TrackersManagerTester::position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &msg) {}

void TrackersManagerTester::tracker_status_callback(const kr_tracker_msgs::TrackerStatus::ConstPtr &msg) {}

/*
 *	@brief Function to Initialize the tester and to see if the action clients are connected to the servers.
 *				 Also checks if the subscribers are connected to the publishers.
 */
bool TrackersManagerTester::initial_checks()
{
  std::lock_guard<std::mutex> lock(mutex);
  bool flag1, flag2, flag3, temp1, temp2;
  distance_client_->waitForServer();
  min_jerk_client_->waitForServer();
  temp1 = distance_client_->isServerConnected();
  ros::Duration(0.5).sleep();
  temp2 = min_jerk_client_->isServerConnected();
  flag1 = temp1 && temp2;

  temp1 = position_cmd_sub_.getNumPublishers() > 0;
  ros::Duration(0.5).sleep();
  temp2 = tracker_status_sub_.getNumPublishers() > 0;
  flag2 = temp1 && temp2;

  flag3 = transition_client_.exists();

  return flag1 && flag2 && flag3;
}

void TrackersManagerTester::send_transition_request(std::string tracker_name)
{
  std::lock_guard<std::mutex> lock(mutex);
  kr_tracker_msgs::Transition msg;
  msg.request.tracker = tracker_name;

  if(transition_client_.call(msg))
  {
    srv_succeed = true;
    srv_response = msg.response.success;
    srv_msg = msg.response.message;
  }
  else
  {
    srv_succeed = false;
  }
}

/*
 * @brief: Struct to store reference data for Test2
 */
struct Test2Data
{
  bool srv_response_success[3] = {false, false, false};
  std::string srv_response_msg[3] = {"Cannot find tracker LineTrackerDistance, cannot transition",
                                     "Failed to activate tracker kr_trackers/LineTrackerDistance, cannot transition",
                                     "Failed to activate tracker kr_trackers/LineTrackerMinJerk, cannot transition"};
};

#endif  // TESTER_UTILS_HPP
