#ifndef MAV_MANAGER_SERVICES_H
#define MAV_MANAGER_SERVICES_H

#include <kr_mav_manager/Circle.h>
#include <kr_mav_manager/CompoundLissajous.h>
#include <kr_mav_manager/GoalTimed.h>
#include <kr_mav_manager/Lissajous.h>
#include <kr_mav_manager/Vec4.h>
#include <kr_mav_manager/manager.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace kr_mav_manager
{
class MAVManagerServices
{
 public:
  std::vector<ros::ServiceServer> srvs_;

  bool motors_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    res.success = mav->set_motors(req.data);
    res.message = "Motors ";
    res.message += req.data ? "on" : "off";
    if(res.success)
      last_cb_ = "motors";
    return true;
  }
  bool takeoff_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->takeoff();
    res.message = "Takeoff";
    if(res.success)
      last_cb_ = "takeoff";
    return true;
  }
  bool goHome_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->goHome();
    res.message = "Going home";
    if(res.success)
      last_cb_ = "goHome";
    return true;
  }
  bool goTo_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res)
  {
    res.success = mav->goTo(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.message = "Going To...";
    if(res.success)
      last_cb_ = "goTo";
    return true;
  }
  bool goToTimed_cb(kr_mav_manager::GoalTimed::Request &req, kr_mav_manager::GoalTimed::Response &res)
  {
    res.success = mav->goToTimed(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, false, req.duration,
                                 req.t_start);
    res.message = "Going To Timed...";
    if(res.success)
      last_cb_ = "goToTimed";
    return res.success;
  }
  bool goToRelative_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res)
  {
    res.success = mav->goTo(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, true);
    res.message = "Going To Relative Position...";
    if(res.success)
      last_cb_ = "goToRelative";
    return res.success;
  }
  bool setDesVelInWorldFrame_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res)
  {
    res.success = mav->setDesVelInWorldFrame(req.goal[0], req.goal[1], req.goal[2], req.goal[3], true);
    res.message = "World Velocity";
    if(res.success)
      last_cb_ = "setDesVelInWorldFrmae";
    return true;
  }
  bool setDesVelInBodyFrame_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res)
  {
    res.success = mav->setDesVelInBodyFrame(req.goal[0], req.goal[1], req.goal[2], req.goal[3], true);
    res.message = "Body Velocity";
    if(res.success)
      last_cb_ = "setDesVelInBodyFrame";
    return true;
  }
  bool circle_cb(kr_mav_manager::Circle::Request &req, kr_mav_manager::Circle::Response &res)
  {
    res.success = mav->circle(req.Ax, req.Ay, req.T, req.duration);
    res.message = "Circling motion";
    if(res.success)
      last_cb_ = "circle";
    return true;
  }
  bool lissajous_cb(kr_mav_manager::Lissajous::Request &req, kr_mav_manager::Lissajous::Response &res)
  {
    res.success = mav->lissajous(req.x_amp, req.y_amp, req.z_amp, req.yaw_amp, req.x_num_periods, req.y_num_periods,
                                 req.z_num_periods, req.yaw_num_periods, req.period, req.num_cycles, req.ramp_time);
    res.message = "Lissajous motion";
    if(res.success)
      last_cb_ = "lissajous";
    return true;
  }
  bool compound_lissajous_cb(kr_mav_manager::CompoundLissajous::Request &req,
                             kr_mav_manager::CompoundLissajous::Response &res)
  {
    float x_amp[2] = {req.x_amp[0], req.x_amp[1]};
    float y_amp[2] = {req.y_amp[0], req.y_amp[1]};
    float z_amp[2] = {req.z_amp[0], req.z_amp[1]};
    float yaw_amp[2] = {req.yaw_amp[0], req.yaw_amp[1]};
    float x_num_periods[2] = {req.x_num_periods[0], req.x_num_periods[1]};
    float y_num_periods[2] = {req.y_num_periods[0], req.y_num_periods[1]};
    float z_num_periods[2] = {req.z_num_periods[0], req.z_num_periods[1]};
    float yaw_num_periods[2] = {req.yaw_num_periods[0], req.yaw_num_periods[1]};
    float period[2] = {req.period[0], req.period[1]};
    float num_cycles[2] = {req.num_cycles[0], req.num_cycles[1]};
    float ramp_time[2] = {req.ramp_time[0], req.ramp_time[1]};
    res.success = mav->compound_lissajous(x_amp, y_amp, z_amp, yaw_amp, x_num_periods, y_num_periods, z_num_periods,
                                          yaw_num_periods, period, num_cycles, ramp_time);
    res.message = "Compound Lissajous motion";
    if(res.success)
      last_cb_ = "compound_lissajous";
    return true;
  }
  bool hover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->hover();
    res.message = "Hover";
    if(res.success)
      last_cb_ = "hover";
    return true;
  }
  bool ehover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->ehover();
    res.message = "Emergency Hover";
    if(res.success)
      last_cb_ = "ehover";
    return true;
  }
  bool land_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->land();
    res.message = "Landing";
    if(res.success)
      last_cb_ = "land";
    return true;
  }
  bool eland_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->eland();
    res.message = "Emergency Landing";
    if(res.success)
      last_cb_ = "eland";
    return true;
  }
  bool estop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    res.success = mav->estop();
    res.message = "Emergency Stop";
    if(res.success)
      last_cb_ = "estop";
    return true;
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
    srvs_.push_back(nh_.advertiseService("circle", &MAVManagerServices::circle_cb, this));
    srvs_.push_back(nh_.advertiseService("lissajous", &MAVManagerServices::lissajous_cb, this));
    srvs_.push_back(nh_.advertiseService("compound_lissajous", &MAVManagerServices::compound_lissajous_cb, this));
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
}  // namespace kr_mav_manager
#endif /* MAV_MANAGER_SERVICES_H */
