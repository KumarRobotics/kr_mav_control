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
  void send_action_goal(std::string tracker_name, float x, float y, float z, float yaw, float v_des, float a_des,
                        uint8_t relative);
  void publish_odom_msg(int secs, int nsecs, float pos_x, float pos_y, float pos_z, float orient_x, float orient_y,
                        float orient_z, float orient_w);
  bool srv_response;
  std::string srv_msg;
  bool srv_succeed = false;
  bool result_received = false;
  bool feedback_received = false;
  kr_tracker_msgs::LineTrackerResultPtr action_result;
  kr_tracker_msgs::LineTrackerFeedbackPtr action_feedback;
  bool position_cmd_received = false;
  kr_mav_msgs::PositionCommandPtr cmd;
  bool tracker_status_received = false;
  kr_tracker_msgs::TrackerStatusPtr status;
  std::mutex mutex;

 private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber position_cmd_sub_, tracker_status_sub_;
  ros::ServiceClient transition_client_;

  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> ClientType;
  std::shared_ptr<ClientType> distance_client_;
  std::shared_ptr<ClientType> min_jerk_client_;
  void done_callback(const actionlib::SimpleClientGoalState &state,
                     const kr_tracker_msgs::LineTrackerResultConstPtr &result);
  void feedback_callback(const kr_tracker_msgs::LineTrackerFeedbackConstPtr &feedback);
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

void TrackersManagerTester::publish_odom_msg(int secs, int nsecs, float pos_x, float pos_y, float pos_z, float orient_x,
                                             float orient_y, float orient_z, float orient_w)
{
  std::lock_guard<std::mutex> lock(mutex);

  nav_msgs::Odometry msg;
  msg.header.stamp.sec = secs;
  msg.header.stamp.nsec = nsecs;
  msg.pose.pose.position.x = pos_x;
  msg.pose.pose.position.y = pos_y;
  msg.pose.pose.position.z = pos_z;
  msg.pose.pose.orientation.x = orient_x;
  msg.pose.pose.orientation.y = orient_y;
  msg.pose.pose.orientation.z = orient_z;
  msg.pose.pose.orientation.w = orient_w;

  odom_pub_.publish(msg);
}

void TrackersManagerTester::position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  if(cmd)
  {
    cmd.reset();
    cmd = boost::make_shared<kr_mav_msgs::PositionCommand>(*msg);
  }
  else
  {
    cmd = boost::make_shared<kr_mav_msgs::PositionCommand>(*msg);
  }
  position_cmd_received = true;
}

void TrackersManagerTester::tracker_status_callback(const kr_tracker_msgs::TrackerStatus::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  if(status)
  {
    status.reset();
    status = boost::make_shared<kr_tracker_msgs::TrackerStatus>(*msg);
  }
  else
  {
    status = boost::make_shared<kr_tracker_msgs::TrackerStatus>(*msg);
  }
  tracker_status_received = true;
}

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

void TrackersManagerTester::send_action_goal(std::string tracker_name, float x, float y, float z, float yaw,
                                             float v_des, float a_des, uint8_t relative)
{
  std::lock_guard<std::mutex> lock(mutex);
  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x = x;
  goal.y = y;
  goal.z = z;
  goal.yaw = yaw;
  goal.v_des = v_des;
  goal.a_des = a_des;
  goal.relative = relative;

  if(tracker_name == std::string("kr_trackers/LineTrackerDistance"))
  {
    distance_client_->sendGoal(goal, boost::bind(&TrackersManagerTester::done_callback, this, _1, _2),
                               ClientType::SimpleActiveCallback(),
                               boost::bind(&TrackersManagerTester::feedback_callback, this, _1));
  }
  else if(tracker_name == std::string("kr_trackers/LineTrackerMinJerk"))
  {
    min_jerk_client_->sendGoal(goal, boost::bind(&TrackersManagerTester::done_callback, this, _1, _2),
                               ClientType::SimpleActiveCallback(),
                               boost::bind(&TrackersManagerTester::feedback_callback, this, _1));
  }
  result_received = false;
  feedback_received = false;
}

void TrackersManagerTester::done_callback(const actionlib::SimpleClientGoalState &state,
                                          const kr_tracker_msgs::LineTrackerResultConstPtr &result)
{
  std::lock_guard<std::mutex> lock(mutex);
  if(action_result)
  {
    action_result.reset();
    action_result = boost::make_shared<kr_tracker_msgs::LineTrackerResult>(*result);
  }
  else
  {
    action_result = boost::make_shared<kr_tracker_msgs::LineTrackerResult>(*result);
  }
  result_received = true;
}

void TrackersManagerTester::feedback_callback(const kr_tracker_msgs::LineTrackerFeedbackConstPtr &feedback)
{
  std::lock_guard<std::mutex> lock(mutex);
  if(action_feedback)
  {
    action_feedback.reset();
    action_feedback = boost::make_shared<kr_tracker_msgs::LineTrackerFeedback>(*feedback);
  }
  else
  {
    action_feedback = boost::make_shared<kr_tracker_msgs::LineTrackerFeedback>(*feedback);
  }
  feedback_received = true;
}

/*
 * @brief Struct to store reference data for Test2
 */
struct Test2Data
{
  bool srv_response_success[3] = {false, false, false};
  std::string srv_response_msg[3] = {"Cannot find tracker LineTrackerDistance, cannot transition",
                                     "Failed to activate tracker kr_trackers/LineTrackerDistance, cannot transition",
                                     "Failed to activate tracker kr_trackers/LineTrackerMinJerk, cannot transition"};
};

/*
 * @brief Struct to store data for Test3.
 *        Send a goal, activate tracker and send odom messages to reach goal.
 *        A value of -100 indicates no need to check that value
 */
struct Test3Data
{
  int odom_secs[7] = {0, 0, 0, 0, 0, 0, 0};
  int odom_nsecs[7] = {100000000, 200000000, 300000000, 400000000, 500000000, 600000000, 700000000};
  float odom_pos_x[7] = {0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0};
  float odom_pos_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float odom_pos_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float odom_orient_x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float odom_orient_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float odom_orient_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float odom_orient_w[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  float feedback[7] = {1.0, 0.89999, 0.69999, 0.5, 0.3, 0.1, -100.0};

  float cmd_pos_x[7] = {0.005, 0.1497213, 0.38245967, 0.595, 0.77245968, 0.939721345, 1.0};
  float cmd_pos_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_pos_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_vel_x[7] = {0.0, 0.44721359, 0.77459669, 1.0, 0.77459669, 0.447213649, 0.0};
  float cmd_vel_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_vel_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_accel_x[7] = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 0.0};
  float cmd_accel_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_accel_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_jerk_x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_jerk_y[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float cmd_jerk_z[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double cmd_yaw[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double cmd_yawdot[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::string status_tracker[7] = {"kr_trackers/LineTrackerDistance", "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance", "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance", "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance"};
  uint8_t status_status[7] = {0, 0, 0, 0, 0, 0, 1};

  float result_x = 1.0;
  float result_y = 0.0;
  float result_z = 0.0;
  float result_yaw = 0.0;
  float result_duration = 0.7;
  float result_length = 1.0;
  uint8_t result_status = 3;
};

struct Test4Data
{
  int odom_secs[7] = {-100, 1, 1, 1, -100, 1, 1};
  int odom_nsecs[7] = {-100, 000000000, 100000000, 400000000, -100, 600000000, 700000000};
  float odom_pos_x[7] = {-100.0, 1.0, 1.2, 1.6, -100.0, 1.8, 1.94};
  float odom_pos_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float odom_pos_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float odom_orient_x[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float odom_orient_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float odom_orient_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float odom_orient_w[7] = {-100.0, 1.0, 1.0, 1.0, -100.0, 1.0, 1.0};

  float feedback[7] = {-100.0, 2.0, 1.79999, 1.39999, -100.0, 0.2, -100.0};

  float cmd_pos_x[7] = {-100.0, 1.0449999, 1.268245577, 1.9736335277, -100.0, 1.946491, 2.0};
  float cmd_pos_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_pos_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_vel_x[7] = {-100.0, 0.0, 0.632455587, 1.09544515, -100.0, 0.6324554, 0.0};
  float cmd_vel_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_vel_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_accel_x[7] = {-100.0, 1.0, 1.0, 1.0, -100.0, 1.0, 0.0};
  float cmd_accel_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_accel_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_jerk_x[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_jerk_y[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  float cmd_jerk_z[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  double cmd_yaw[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};
  double cmd_yawdot[7] = {-100.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0};

  std::string status_tracker[7] = {"NoCheck",
                                   "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance",
                                   "NoCheck",
                                   "kr_trackers/LineTrackerDistance",
                                   "kr_trackers/LineTrackerDistance"};
  uint8_t status_status[7] = {0, 0, 0, 0, 0, 0, 1};

  float result_x = 2.0;
  float result_y = 0.0;
  float result_z = 0.0;
  float result_yaw = 0.0;
  float result_duration = 0.3;
  float result_length = 0.34;
  uint8_t result_status = 3;
};

#endif  // TESTER_UTILS_HPP
