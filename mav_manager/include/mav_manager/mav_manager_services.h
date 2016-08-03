#ifndef MAV_MANAGER_SERVICES_H
#define MAV_MANAGER_SERVICES_H

#include <mav_manager/manager.h>
#include <mav_manager/Bool.h>
#include <mav_manager/Trigger.h>
#include <mav_manager/Vec4.h>
#include <mav_manager/GoalTimed.h>

class MAVManagerServices
{
  public:

    std::vector<ros::ServiceServer> srvs_;

    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      res.success = mav->set_motors(req.b);
      res.message = "Motors ";
      res.message += req.b ? "on" : "off";
      if (res.success)
        last_cb_ = "motors";
      return res.success;
    }
    bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->takeoff();
      res.message = "Takeoff";
      if (res.success)
        last_cb_ = "takeoff";
      return res.success;
    }
    bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->goHome();
      res.message = "Going home";
      if (res.success)
        last_cb_ = "goHome";
      return res.success;
    }
    bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      res.success = mav->goTo(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
      res.message = "Going To...";
      if (res.success)
        last_cb_ = "goTo";
      return res.success;
    }
    bool goToTimed_cb(mav_manager::GoalTimed::Request &req, mav_manager::GoalTimed::Response &res)
    {
      res.success = mav->goToTimed(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, false, req.duration, req.t_start);
      res.message = "Going To Timed...";
      if (res.success)
        last_cb_ = "goToTimed";
      return res.success;
    }
    bool goToRelative_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      res.success = mav->goTo(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, true);
      res.message = "Going To Relative Position...";
      if (res.success)
        last_cb_ = "goToRelative";
      return res.success;
    }
    bool setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      res.success = mav->setDesVelInWorldFrame(req.goal[0], req.goal[1], req.goal[2], req.goal[3], true);
      res.message = "World Velocity";
      if (res.success)
        last_cb_ = "setDesVelInWorldFrmae";
      return res.success;
    }
    bool setDesVelInBodyFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      res.success = mav->setDesVelInBodyFrame(req.goal[0], req.goal[1], req.goal[2], req.goal[3], true);
      res.message = "Body Velocity";
      if (res.success)
        last_cb_ = "setDesVelInBodyFrame";
      return res.success;
    }
    bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->hover();
      res.message = "Hover";
      if (res.success)
        last_cb_ = "hover";
      return res.success;
    }
    bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->ehover();
      res.message = "Emergency Hover";
      if (res.success)
        last_cb_ = "ehover";
      return res.success;
    }
    bool land_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->land();
      res.message = "Landing";
      if (res.success)
        last_cb_ = "land";
      return res.success;
    }
    bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->eland();
      res.message = "Emergency Landing";
      if (res.success)
        last_cb_ = "eland";
      return res.success;
    }
    bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->estop();
      res.message = "Emergency Stop";
      if (res.success)
        last_cb_ = "estop";
      return res.success;
    }

    // Constructor
    MAVManagerServices(std::shared_ptr<MAVManager> m) : nh_("~"), mav(m), last_cb_("")
    {
      srvs_.push_back(nh_.advertiseService("motors", &MAVManagerServices::motors_cb, this));
      srvs_.push_back(nh_.advertiseService("takeoff", &MAVManagerServices::takeoff_cb, this));
      srvs_.push_back(nh_.advertiseService("goHome", &MAVManagerServices::goHome_cb, this));
      srvs_.push_back(nh_.advertiseService("goTo", &MAVManagerServices::goTo_cb, this));
      srvs_.push_back(nh_.advertiseService("goToTimed", &MAVManagerServices::goToTimed_cb, this));
      srvs_.push_back(nh_.advertiseService("goToRelative", &MAVManagerServices::goToRelative_cb, this));
      srvs_.push_back(nh_.advertiseService("setDesVelInWorldFrame", &MAVManagerServices::setDesVelInWorldFrame_cb, this));
      srvs_.push_back(nh_.advertiseService("setDesVelInBodyFrame", &MAVManagerServices::setDesVelInBodyFrame_cb, this));
      srvs_.push_back(nh_.advertiseService("hover", &MAVManagerServices::hover_cb, this));
      srvs_.push_back(nh_.advertiseService("ehover", &MAVManagerServices::ehover_cb, this));
      srvs_.push_back(nh_.advertiseService("land", &MAVManagerServices::land_cb, this));
      srvs_.push_back(nh_.advertiseService("eland", &MAVManagerServices::eland_cb, this));
      srvs_.push_back(nh_.advertiseService("estop", &MAVManagerServices::estop_cb, this));
    }

  protected:

    ros::NodeHandle nh_;

    // Let's make an MAV pointer
    std::shared_ptr<MAVManager> mav;

    std::string last_cb_;
};

#endif /* MAV_MANAGER_SERVICES_H */
