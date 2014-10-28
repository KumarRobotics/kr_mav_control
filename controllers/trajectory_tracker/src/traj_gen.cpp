#include <iostream>
#include <trajectory_tracker/trajectory_base.h>
#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>



class TrajectoryTracker : public controllers_manager::Controller {
public:
	TrajectoryTracker();

	void Initialize(const ros::NodeHandle &nh);
	bool Activate();
	void Deactivate();

	const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

	void optimizationThread();
private:
	bool is_active_, got_odom_, path_updated_, traj_running_, run_optim_ , optim_done_;
	void pathCB(const nav_msgs::Path::ConstPtr &msg);
	ros::Subscriber path_sub_;

	Mat4Vec wayPoints_;
	Vec4 curr_pose_;
	Vec4 init_pose_;

	const ros::NodeHandle *nh_;

	std::shared_ptr<Trajectory> trajectory_;
	std::shared_ptr<GRBEnv> grb_env_;

	ros::Time start_time_;

	double kx_[3], kv_[3];
};
TrajectoryTracker::TrajectoryTracker(): is_active_(false), got_odom_(false), path_updated_(false), run_optim_(false), optim_done_(false), traj_running_(false) {
	try{
		grb_env_.reset(new GRBEnv);
	}
	catch(GRBException e) {
		ROS_ERROR("%s",e.getMessage().c_str());
	// cout << "Error code = " << e.getErrorCode() << endl;
	// cout << e.getMessage() << endl;
	} 
}
void TrajectoryTracker::Initialize(const ros::NodeHandle &nh) {
	nh.param("gains/pos/x", kx_[0], 2.5);
	nh.param("gains/pos/y", kx_[1], 2.5);
	nh.param("gains/pos/z", kx_[2], 5.0);
	nh.param("gains/vel/x", kv_[0], 2.2);
	nh.param("gains/vel/y", kv_[1], 2.2);
	nh.param("gains/vel/z", kv_[2], 4.0);

	nh_ = &nh;

	ros::NodeHandle private_nh(nh,"trajectory_tracker");
	path_sub_ = private_nh.subscribe("~path", 10, &TrajectoryTracker::pathCB, this, ros::TransportHints().tcpNoDelay());

}
bool TrajectoryTracker::Activate(){
	if(got_odom_) {
		is_active_ = true;
	}
	return is_active_;
}
const quadrotor_msgs::PositionCommand::Ptr TrajectoryTracker::update(const nav_msgs::Odometry::ConstPtr &msg) {
	ROS_ERROR("calling back");
	curr_pose_(0) = msg->pose.pose.position.x;
	curr_pose_(1) = msg->pose.pose.position.y;
	curr_pose_(2) = msg->pose.pose.position.z;
	curr_pose_(3) = tf::getYaw(msg->pose.pose.orientation);

	if(!got_odom_) {
		init_pose_ = curr_pose_;
		got_odom_ = true;
	}
	if(!is_active_){
    	return quadrotor_msgs::PositionCommand::Ptr();
	}
	// cmd setup
  	quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
	cmd->header.stamp = ros::Time::now();
	cmd->header.frame_id = msg->header.frame_id;

  	cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  	cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];
	
	// run optimization?
	if(path_updated_) {
		run_optim_ = true;
		optimizationThread();
		path_updated_ = false;
	}
	else if(optim_done_){
		if(!traj_running_) {
			start_time_ = msg->header.stamp;
			traj_running_ = true;
		}
		decimal_t dt = (msg->header.stamp - start_time_).toSec();
		Vec4 pos, vel, acc, jrk;
		trajectory_->evaluate(dt,0,pos);
		trajectory_->evaluate(dt,1,vel);
		trajectory_->evaluate(dt,2,acc);
		trajectory_->evaluate(dt,3,jrk);
		
		cmd->position.x = pos(0); cmd->position.y = pos(1); cmd->position.z = pos(2);
		cmd->velocity.x = vel(0); cmd->velocity.y = vel(1); cmd->velocity.z = vel(2);
		cmd->acceleration.x = acc(0); cmd->acceleration.y = acc(1); cmd->acceleration.z = acc(2);
		cmd->jerk.x = jrk(0); cmd->jerk.y = jrk(1); cmd->jerk.z = jrk(2);

		cmd->yaw = pos(3);
		cmd->yaw_dot = vel(3);

		return cmd;
	}

	// pack command_pose into position command pointer
 	cmd->position.x = init_pose_(0); cmd->position.y = init_pose_(1); cmd->position.z = init_pose_(2);
	cmd->velocity.x = 0; cmd->velocity.y = 0; cmd->velocity.z = 0;
	cmd->acceleration.x = 0; cmd->acceleration.y = 0; cmd->acceleration.z = 0;
	cmd->jerk.x = 0; cmd->jerk.y = 0; cmd->jerk.z = 0;
  	
  	cmd->yaw = init_pose_(3);
  	cmd->yaw_dot = 0;
}
void TrajectoryTracker::pathCB(const nav_msgs::Path::ConstPtr &msg){
	wayPoints_.clear();
	for(auto &pose : msg->poses) {
		Mat4 curr = Mat4::Ones() * NAN; // make nan
		
		// pack in pose
		curr(0,0) = pose.pose.position.x;
		curr(1,0) = pose.pose.position.y;
		curr(2,0) = pose.pose.position.z;
		curr(3,0) = tf::getYaw(pose.pose.orientation);

		wayPoints_.push_back(curr);
	}
	path_updated_ = true;
}
void TrajectoryTracker::optimizationThread() {
	// while(nh_->oh()) {
		// this.sleep(50);
		if(run_optim_) {
			// create new array and append current pose to front
			Mat4Vec waypnts;
			Mat4 curr_aug = Mat4::Zero();
			curr_aug.block<4,1>(0,0) = curr_pose_; // assume start from zero velocity
			waypnts.push_back(curr_aug);
			waypnts.insert(waypnts.end(),wayPoints_.begin(),wayPoints_.end());

			std::vector<decimal_t> dts(waypnts.size(),0.5);
			try {
				GRBModel model(*grb_env_);
				trajectory_.reset(new Trajectory(&model,waypnts,dts));
				trajectory_->addMaximumBound(1.0,1);
				trajectory_->addMaximumBound(2.0,2);
				trajectory_->addMaximumBound(3.0,3);
				
				model.optimize();

				trajectory_->recoverVars();

				optim_done_ = true;
			}
			catch(...) {
				ROS_ERROR("Exception during optimization");
			}

			run_optim_ = false;
		}
	// }
}
void TrajectoryTracker::Deactivate() {
	is_active_ = false;
	got_odom_ = false;
}
// int main() {
// 	std::cout << "Im a program!!" << std::endl;
// }
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrajectoryTracker, controllers_manager::Controller);
