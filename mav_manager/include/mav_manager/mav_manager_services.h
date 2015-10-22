#ifndef MAV_MANAGER_SERVICES_H
#define MAV_MANAGER_SERVICES_H

#include <mav_manager/manager.h>
#include <mav_manager/Bool.h>
#include <mav_manager/Trigger.h>
#include <mav_manager/Vec4.h>

// Typedefs
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;

class MAVManagerServices
{
  public:

    std::vector<ros::ServiceServer> srvs_;

    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      res.success = mav->set_motors(req.b);
      res.message = "Motors ";
      res.message += req.b ? "on" : "off";
      return res.success;
    }
    bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->takeoff();
      res.message = "Takeoff";
      return res.success;
    }
    bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->goHome();
      res.message = "Going home";
      return res.success;
    }
    bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
      res.success = mav->goTo(goal);
      res.message = "Going To...";
      return res.success;
    }
    bool setDesVelWorld_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
      res.success = mav->setDesVelWorld(goal);
      res.message = "World Velocity";
      return res.success;
    }
    bool setDesVelBody_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
      res.success = mav->setDesVelBody(goal);
      res.message = "Body Velocity";
      return res.success;
    }
    bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->hover();
      res.message = "Hover";
      return res.success;
    }
    bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->ehover();
      res.message = "Emergency Hover";
      return res.success;
    }
    bool land_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->land();
      res.message = "Landing";
      return res.success;
    }
    bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->eland();
      res.message = "Emergency Landing";
      return res.success;
    }
    bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->estop();
      res.message = "Emergency Stop";
      return res.success;
    }

    // Constructor
    MAVManagerServices(std::shared_ptr<MAVManager> m) : nh_("mav_manager"), nh_priv_("~"), mav(m)
    {
      srvs_.push_back(nh_.advertiseService("motors", &MAVManagerServices::motors_cb, this));
      srvs_.push_back(nh_.advertiseService("takeoff", &MAVManagerServices::takeoff_cb, this));
      srvs_.push_back(nh_.advertiseService("goHome", &MAVManagerServices::goHome_cb, this));
      srvs_.push_back(nh_.advertiseService("goTo", &MAVManagerServices::goTo_cb, this));
      srvs_.push_back(nh_.advertiseService("setDesVelWorld", &MAVManagerServices::setDesVelWorld_cb, this));
      srvs_.push_back(nh_.advertiseService("setDesVelBody", &MAVManagerServices::setDesVelBody_cb, this));
      srvs_.push_back(nh_.advertiseService("hover", &MAVManagerServices::hover_cb, this));
      srvs_.push_back(nh_.advertiseService("ehover", &MAVManagerServices::ehover_cb, this));
      srvs_.push_back(nh_.advertiseService("land", &MAVManagerServices::land_cb, this));
      srvs_.push_back(nh_.advertiseService("eland", &MAVManagerServices::eland_cb, this));
      srvs_.push_back(nh_.advertiseService("estop", &MAVManagerServices::estop_cb, this));
    }

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    // Let's make an MAV pointer
    std::shared_ptr<MAVManager> mav;
};

#endif /* MAV_MANAGER_SERVICES_H */
