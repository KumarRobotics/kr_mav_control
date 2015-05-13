#include <ros/ros.h>
#include <manager.h>

// Services
ros::ServiceServer
  srv_motors_,
  srv_takeoff_,
  srv_goTo_,
  srv_setDesVelWorld_,
  srv_setDesVelBody_,
  srv_hover_,
  srv_ehover_,
  srv_estop_;

// Typedefs
typedef Eigen::Vector3d    vec3;
typedef Eigen::Vector4d    vec4;
typedef Eigen::Quaterniond quat;

class MAV_Services
{
  public:

  // Let's make an MAV
  MAVManager mav_;

  bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
  {
    mav_.motors(req.flag);
    res.success = true;
    return true;
  }
  bool takeoff_cb(mav_manager::Empty::Request &req, mav_manager::Empty::Response &res)
  {
    ROS_INFO("Trying to takeoff");
    bool flag = mav_.takeoff();
    res.success = flag;
    return flag;
  }
  bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    bool flag = mav_.goTo(goal);
    res.success = flag;
    return flag;
  }
  bool setDesVelWorld_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    mav_.setDesVelWorld(goal);
    return true;
  }
  bool setDesVelBody_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    mav_.setDesVelBody(goal);
    return true;
  }
  bool hover_cb(mav_manager::Empty::Request &req, mav_manager::Empty::Response &res)
  {
    bool flag = mav_.hover();
    res.success = flag;
    return flag;
  }
  bool ehover_cb(mav_manager::Empty::Request &req, mav_manager::Empty::Response &res)
  {
    bool flag = mav_.ehover();
    res.success = flag;
    return flag;
  }
  bool estop_cb(mav_manager::Empty::Request &req, mav_manager::Empty::Response &res)
  {
    mav_.estop();
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh("");

  MAV_Services mav_srvs;

  // Services
  srv_motors_ = nh.advertiseService("motors", &MAV_Services::motors_cb, &mav_srvs);
  srv_takeoff_ = nh.advertiseService("takeoff", &MAV_Services::takeoff_cb, &mav_srvs);
  srv_goTo_ = nh.advertiseService("goTo", &MAV_Services::goTo_cb, &mav_srvs);
  srv_setDesVelWorld_ = nh.advertiseService("setDesVelWorld", &MAV_Services::setDesVelWorld_cb, &mav_srvs);
  srv_setDesVelBody_ = nh.advertiseService("setDesVelBody", &MAV_Services::setDesVelBody_cb, &mav_srvs);
  srv_hover_ = nh.advertiseService("hover", &MAV_Services::hover_cb, &mav_srvs);
  srv_ehover_ = nh.advertiseService("ehover", &MAV_Services::ehover_cb, &mav_srvs);
  srv_estop_ = nh.advertiseService("estop", &MAV_Services::estop_cb, &mav_srvs);

  // Let's spin some rotors
  ros::spin();

  return 0;
}