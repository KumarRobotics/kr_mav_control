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
  srv_eland_,
  srv_estop_;

// Typedefs
typedef Eigen::Vector4d    Vec4;

class MAV_Services
{
  public:

  // Let's make an MAV
  MAVManager mav_;

  bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
  {
    mav_.motors(req.b);
    res.success = true;
    res.message = "Motors activated";
    return true;
  }
  bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.takeoff();
    res.message = "Takeoff";
    return res.success;
  }
  bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.goTo(goal);
    res.message = "Go To";
    return res.success;
  }
  bool setDesVelWorld_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.setDesVelWorld(goal);
    res.message = "World Velocity";
    return res.success;
  }
  bool setDesVelBody_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.setDesVelBody(goal);
    res.message = "Body Velocity";
    return res.success;
  }
  bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.hover();
    res.message = "Hover";
    return res.success;
  }
  bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.ehover();
    res.message = "Emergency Hover";
    return res.success;
  }
  bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.eland();
    res.message = "Emergency Landing";
    return res.success;
  }
  bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    mav_.estop();
    res.success = true;
    res.message = "Emergency Stop";
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
  srv_eland_ = nh.advertiseService("eland", &MAV_Services::eland_cb, &mav_srvs);
  srv_estop_ = nh.advertiseService("estop", &MAV_Services::estop_cb, &mav_srvs);

  // Let's spin some rotors
  ros::spin();

  return 0;
}
