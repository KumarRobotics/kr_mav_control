#pragma once

#include "kr_mav_manager/srv/circle.hpp"
#include "kr_mav_manager/srv/compound_lissajous.hpp"
#include "kr_mav_manager/srv/goal_timed.hpp"
#include "kr_mav_manager/srv/lissajous.hpp"
#include "kr_mav_manager/srv/vec4.hpp"
#include "kr_mav_manager/manager.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace kr_mav_manager
{

using namespace std::placeholders;

class MAVManagerServices
{
public:

  void motors_cb(const std_srvs::srv::SetBool::Request::SharedPtr req, 
                 const std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    res->success = mav->set_motors(req->data);
    res->message = "Motors ";
    res->message += req->data ? "on" : "off";
    if(res->success)
      last_cb_ = "motors";
  }

  void takeoff_cb(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                  const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->takeoff();
    res->message = "Takeoff";
    if(res->success)
      last_cb_ = "takeoff";
  }

  void goHome_cb(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->goHome();
    res->message = "Going Home";
    if(res->success)
      last_cb_ = "goHome";
  }

  void goTo_cb(const kr_mav_manager::srv::Vec4::Request::SharedPtr req,
               const kr_mav_manager::srv::Vec4::Response::SharedPtr res)
  {
    res->success = mav->goTo(req->goal[0], req->goal[1], req->goal[2], req->goal[4]);
    res->message = "Going To...";
    if(res->success)
      last_cb_ = "goTo";
  }

  void goToTimed_cb(const kr_mav_manager::srv::GoalTimed::Request::SharedPtr req,
                    const kr_mav_manager::srv::GoalTimed::Response::SharedPtr res)
  {
    res->success = mav->goToTimed(req->goal[0], req->goal[1], req->goal[2], req->goal[3],
                                  0.0f, 0.0f, false, req->duration, req->t_start);
    res->message = "Going To Timed...";
    if(res->success)
      last_cb_ = "goToTimed";
  }

  void goToRelative_cb(const kr_mav_manager::srv::Vec4::Request::SharedPtr req,
                       const kr_mav_manager::srv::Vec4::Response::SharedPtr res)
  {
    res->success = mav->goTo(req->goal[0], req->goal[1], req->goal[2], req->goal[3], 0.0f, 0.0f, true);
    res->message = "Going To Relative Position...";
    if(res->success)
      last_cb_ = "goToRelative";
  }

  void setDesVelInWorldFrame_cb(const kr_mav_manager::srv::Vec4::Request::SharedPtr req,
                             const kr_mav_manager::srv::Vec4::Response::SharedPtr res)
  {
    res->success = mav->setDesVelInWorldFrame(req->goal[0], req->goal[1], req->goal[2], req->goal[3], true);
    res->message = "World Velocity";
    if(res->success)
      last_cb_ = "setDesVelInWorldFrame";
  }

  void setDesVelInBodyFrame_cb(const kr_mav_manager::srv::Vec4::Request::SharedPtr req,
                            const kr_mav_manager::srv::Vec4::Response::SharedPtr res)
  {
    res->success = mav->setDesVelInBodyFrame(req->goal[0], req->goal[1], req->goal[2], req->goal[3], true);
    res->message = "Body Velocity";
    if(res->success)
      last_cb_ = "setDesVelInBodyFrame";
  }

  void circle_cb(const kr_mav_manager::srv::Circle::Request::SharedPtr req,
                 const kr_mav_manager::srv::Circle::Response::SharedPtr res)
  {
    res->success = mav->circle(req->ax, req->ay, req->t, req->duration);
    res->message = "Circling motion";
    if(res->success)
      last_cb_ = "circle";
  }

  void lissajous_cb(const kr_mav_manager::srv::Lissajous::Request::SharedPtr req,
                    const kr_mav_manager::srv::Lissajous::Response::SharedPtr res)
  {
    res->success = mav->lissajous(req->x_amp, req->y_amp, req->z_amp, req->y_amp,
                                  req->x_num_periods, req->y_num_periods, req->z_num_periods,
                                  req->yaw_num_periods, req->period, req->num_cycles, req->ramp_time);
    res->message = "Lissajous motion";
    if(res->success)
      last_cb_ = "lissajous";
  }

  void compound_lissajous_cb(const kr_mav_manager::srv::CompoundLissajous::Request::SharedPtr req,
                             const kr_mav_manager::srv::CompoundLissajous::Response::SharedPtr res)
  {
    float x_amp[2] = {req->x_amp[0], req->x_amp[1]};
    float y_amp[2] = {req->y_amp[0], req->y_amp[1]};
    float z_amp[2] = {req->z_amp[0], req->z_amp[1]};
    float yaw_amp[2] = {req->yaw_amp[0], req->yaw_amp[1]};
    float x_num_periods[2] = {req->x_num_periods[0], req->x_num_periods[1]};
    float y_num_periods[2] = {req->y_num_periods[0], req->y_num_periods[1]};
    float z_num_periods[2] = {req->z_num_periods[0], req->z_num_periods[1]};
    float yaw_num_periods[2] = {req->yaw_num_periods[0], req->yaw_num_periods[1]};
    float period[2] = {req->period[0], req->period[1]};
    float num_cycles[2] = {req->num_cycles[0], req->num_cycles[1]};
    float ramp_time[2] = {req->ramp_time[0], req->ramp_time[1]};
    res->success = mav->compound_lissajous(x_amp, y_amp, z_amp, yaw_amp, x_num_periods,
                                           y_num_periods, z_num_periods, yaw_num_periods, 
                                           period, num_cycles, ramp_time);
    res->message = "Compound Lissajous motion";
    if(res->success)
      last_cb_ = "compound_lissajous";
  }

  void hover_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->hover();
    res->message = "Hover";
    if(res->success)
      last_cb_ = "hover";
  }

  void ehover_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->ehover();
    res->message = "Emergency Hover";
    if(res->success)
      last_cb_ = "ehover";
  }

  void land_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
               const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->land();
    res->message = "Landing";
    if(res->success)
      last_cb_ = "land";
  }

  void eland_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->eland();
    res->message = "Emergency Landing";
    if(res->success)
      last_cb_ = "eland";
  }

  void estop_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    (void)req;
    res->success = mav->estop();
    res->message = "Emergency Stop";
    if(res->success)
      last_cb_ = "estop";
  }

    //Constructor
  MAVManagerServices(std::shared_ptr<MAVManager> m) : mav(m), last_cb_("")
  {
    motors_srv_ = mav->create_service<std_srvs::srv::SetBool>("motors",
                       std::bind(&MAVManagerServices::motors_cb, this, _1, _2));
    takeoff_srv_ = mav->create_service<std_srvs::srv::Trigger>("takeoff",
                        std::bind(&MAVManagerServices::takeoff_cb, this, _1, _2));
    goHome_srv_ = mav->create_service<std_srvs::srv::Trigger>("goHome",
                       std::bind(&MAVManagerServices::goHome_cb, this, _1, _2));
    goTo_srv_ = mav->create_service<kr_mav_manager::srv::Vec4>("goTo",
                     std::bind(&MAVManagerServices::goTo_cb, this, _1, _2));
    goToTimed_srv_ = mav->create_service<kr_mav_manager::srv::GoalTimed>("goToTimed",
                          std::bind(&MAVManagerServices::goToTimed_cb, this, _1, _2));
    goToRelative_srv_ = mav->create_service<kr_mav_manager::srv::Vec4>("goToRelative",
                             std::bind(&MAVManagerServices::goToRelative_cb, this, _1, _2));
    setDesVelInWorldFrame_srv_ = mav->create_service<kr_mav_manager::srv::Vec4>("setDesVelInWorldFrame",
                                std::bind(&MAVManagerServices::setDesVelInWorldFrame_cb, this, _1, _2));
    setDesVelInBodyFrame_srv_ = mav->create_service<kr_mav_manager::srv::Vec4>("setDesVelInBodyFrame",
                                std::bind(&MAVManagerServices::setDesVelInBodyFrame_cb, this, _1, _2));
    circle_srv_ = mav->create_service<kr_mav_manager::srv::Circle>("circle",
                       std::bind(&MAVManagerServices::circle_cb, this, _1, _2));
    lissajous_srv_ = mav->create_service<kr_mav_manager::srv::Lissajous>("lissajous",
                          std::bind(&MAVManagerServices::lissajous_cb, this, _1, _2));
    compound_lissajous_srv_ = mav->create_service<kr_mav_manager::srv::CompoundLissajous>("compound_lissajous",
                                   std::bind(&MAVManagerServices::compound_lissajous_cb, this, _1, _2));
    hover_srv_ = mav->create_service<std_srvs::srv::Trigger>("hover",
                      std::bind(&MAVManagerServices::hover_cb, this, _1, _2));
    ehover_srv_ = mav->create_service<std_srvs::srv::Trigger>("ehover",
                       std::bind(&MAVManagerServices::ehover_cb, this, _1, _2));
    land_srv_ = mav->create_service<std_srvs::srv::Trigger>("land",
                     std::bind(&MAVManagerServices::land_cb, this, _1, _2));
    eland_srv_ = mav->create_service<std_srvs::srv::Trigger>("eland",
                      std::bind(&MAVManagerServices::eland_cb, this, _1, _2));
    estop_srv_ = mav->create_service<std_srvs::srv::Trigger>("estop",
                      std::bind(&MAVManagerServices::estop_cb, this, _1, _2));
  }

protected:

  // Making a MAV pointer
  std::shared_ptr<kr_mav_manager::MAVManager> mav;

  std::string last_cb_;

  // Creating Servers
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motors_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goHome_srv_;
  rclcpp::Service<kr_mav_manager::srv::Vec4>::SharedPtr goTo_srv_;
  rclcpp::Service<kr_mav_manager::srv::GoalTimed>::SharedPtr goToTimed_srv_;
  rclcpp::Service<kr_mav_manager::srv::Vec4>::SharedPtr goToRelative_srv_;
  rclcpp::Service<kr_mav_manager::srv::Vec4>::SharedPtr setDesVelInWorldFrame_srv_;
  rclcpp::Service<kr_mav_manager::srv::Vec4>::SharedPtr setDesVelInBodyFrame_srv_;
  rclcpp::Service<kr_mav_manager::srv::Circle>::SharedPtr circle_srv_;
  rclcpp::Service<kr_mav_manager::srv::Lissajous>::SharedPtr lissajous_srv_;
  rclcpp::Service<kr_mav_manager::srv::CompoundLissajous>::SharedPtr compound_lissajous_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ehover_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr eland_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;

};
} // namespace kr_mav_manager