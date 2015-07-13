#include <mav_manager/manager.h>
#include <mav_manager/Bool.h>
#include <mav_manager/Trigger.h>
#include <mav_manager/Vec4.h>


// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Services
{
  public:

    // Services
    ros::ServiceServer
      srv_motors_,
      srv_takeoff_,
      srv_goHome_,
      srv_goTo_,
      srv_setDesVelWorld_,
      srv_setDesVelBody_,
      srv_useRadioForVelocity_,
      srv_hover_,
      srv_ehover_,
      srv_eland_,
      srv_estop_;

    // Let's make an MAV
    MAVManager mav_;

    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      res.success = mav_.motors(req.b);
      res.message = "Motors ";
      res.message += req.b ? "on" : "off";
      return res.success;
    }
    bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav_.takeoff();
      res.message = "Takeoff";
      return res.success;
    }
    bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav_.goHome();
      res.message = "Going home";
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
    bool useRadioForVelocity_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      // TODO: Make this callback trigger a flag and sets the velocity using mav_.setDesVelBody
      // res.success = mav_.useRadioForVelocity(req.b);
      res.success = false;
      if (res.success)
      {
        if (req.b)
          res.message = "Using radio for velocity";
        else
          res.message = "No longer using radio for velocity";
      }
      else
        res.message = "Failed to transition";

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
      res.success = mav_.estop();
      res.message = "Emergency Stop";
      return res.success;
    }

    // Constructor
    MAV_Services() : nh_(""), nh_priv_("~") {

      // Services
      srv_motors_ = nh_.advertiseService("motors", &MAV_Services::motors_cb, this);
      srv_takeoff_ = nh_.advertiseService("takeoff", &MAV_Services::takeoff_cb, this);
      srv_goHome_ = nh_.advertiseService("goHome", &MAV_Services::goHome_cb, this);
      srv_goTo_ = nh_.advertiseService("goTo", &MAV_Services::goTo_cb, this);
      srv_setDesVelWorld_ = nh_.advertiseService("setDesVelWorld", &MAV_Services::setDesVelWorld_cb, this);
      srv_setDesVelBody_ = nh_.advertiseService("setDesVelBody", &MAV_Services::setDesVelBody_cb, this);
      srv_useRadioForVelocity_ = nh_.advertiseService("useRadioForVelocity", &MAV_Services::useRadioForVelocity_cb, this);
      srv_hover_ = nh_.advertiseService("hover", &MAV_Services::hover_cb, this);
      srv_ehover_ = nh_.advertiseService("ehover", &MAV_Services::ehover_cb, this);
      srv_eland_ = nh_.advertiseService("eland", &MAV_Services::eland_cb, this);
      srv_estop_ = nh_.advertiseService("estop", &MAV_Services::estop_cb, this);
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh("");

  MAV_Services mav_srvs;

  // Let's spin some rotors
  ros::spin();

  return 0;
}
