#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Empty.h>
#include <controllers_manager/Transition.h>
#include <std_msgs/Bool.h>


class SamplePlanner {
public:
	SamplePlanner(ros::NodeHandle * nh);
	~SamplePlanner();

private:
	ros::NodeHandle * nh_;
	ros::ServiceServer demo_service_, takeoff_service_;

	ros::Publisher motor_pub_, path_pub_, line_track_pub_;

	// ros::ServiceClient trans_client_;

	bool demoCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool takeoffCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	
	void sendPath();

};
SamplePlanner::SamplePlanner(ros::NodeHandle * nh) : nh_(nh){
	takeoff_service_ = nh_->advertiseService("takeoff", &SamplePlanner::takeoffCB, this);
	demo_service_ = nh_->advertiseService("run_demo_waypoints", &SamplePlanner::demoCB, this);
	motor_pub_ = nh_->advertise<std_msgs::Bool>("motors",5,this);
	line_track_pub_ = nh_->advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker/goal",5,this);
	path_pub_ = nh_->advertise<nav_msgs::Path>("controllers_manager/trajectory_tracker/path",5,this);

	// trans_client_ = nh_->serviceClient<controllers_manager::Transition>("/quadrotor/controllers_manager/transition",this);
}
SamplePlanner::~SamplePlanner() {
}
void SamplePlanner::sendPath() {
	nav_msgs::Path path;
	geometry_msgs::PoseStamped ps;

	ps.pose.orientation.w = 1;
	ps.pose.orientation.x = 0;
	ps.pose.orientation.y = 0;
	ps.pose.orientation.z = 0;

	ps.pose.position.x = 2;
	ps.pose.position.y = 3;
	ps.pose.position.z = 2;

	path.poses.push_back(ps);

	ps.pose.position.x = -1;
	ps.pose.position.y = 4;
	ps.pose.position.z = 5;

	path.poses.push_back(ps);

	ps.pose.position.x = 0;
	ps.pose.position.y = 0;
	ps.pose.position.z = 2;

	path.poses.push_back(ps);

	path_pub_.publish(path);
}

bool SamplePlanner::demoCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	sendPath();
	controllers_manager::Transition trans_srv;
	trans_srv.request.controller = "trajectory_tracker/TrajectoryTracker";
	
	ros::ServiceClient trans_client_ = nh_->serviceClient<controllers_manager::Transition>("/quadrotor/controllers_manager/transition",this);
	
	trans_client_.call(trans_srv);
	return true;
}
bool SamplePlanner::takeoffCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	// cpp translation of takeoff.sh

	std_msgs::Bool bool_msg;
	bool_msg.data = false;

	motor_pub_.publish(bool_msg);

	geometry_msgs::Vector3 goal;
	goal.x = 0; goal.y = 0; goal.z = 2;

	line_track_pub_.publish(goal);

	controllers_manager::Transition trans_srv;
	trans_srv.request.controller = "line_tracker/LineTracker";

	ros::ServiceClient trans_client_ = nh_->serviceClient<controllers_manager::Transition>("/quadrotor/controllers_manager/transition",this);
	if(trans_client_.call(trans_srv)) {
		bool_msg.data = true;
		motor_pub_.publish(bool_msg);
		return true;
	}
	else {
		ROS_ERROR("Couldnt transition controller to take off");
		return false;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_planner");
  ros::NodeHandle nh;
  SamplePlanner planner(&nh);
  ros::spin();
}
