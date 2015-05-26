#include <manager.h>
#include <std_srvs/Empty.h>
#include <mav_manager/Vec4.h>

// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Services {
 public:
  // Services
  ros::ServiceServer srv_motors_, srv_takeoff_, srv_goTo_, srv_setDesVelWorld_,
      srv_setDesVelBody_, srv_useRadioForVelocity_, srv_hover_, srv_ehover_,
      srv_eland_, srv_estop_;

  // Let's make an MAV
  MAVManager mav_;

  bool motors_cb(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
    static bool motor_toggle_ = true;
    mav_.motors(motor_toggle_);
    motor_toggle_ = !motor_toggle_;
    return true;
  }
  bool takeoff_cb(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res) {
    return mav_.takeoff();
  }
  bool goTo_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    Vec4 goal = Vec4::Zero();
    nh_priv_.param("goTo/x", goal(0), 0.0);
    nh_priv_.param("goTo/y", goal(1), 0.0);
    nh_priv_.param("goTo/z", goal(2), 0.0);
    nh_priv_.param("goTo/yaw", goal(3), 0.0);

    return mav_.goTo(goal);
  }
  bool setDesVelWorld_cb(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res) {
    Vec4 goal = Vec4::Zero();
    nh_priv_.param("desVelWorld/x", goal(0), 0.0);
    nh_priv_.param("desVelWorld/y", goal(1), 0.0);
    nh_priv_.param("desVelWorld/z", goal(2), 0.0);
    nh_priv_.param("desVelWorld/yaw", goal(3), 0.0);

    return mav_.setDesVelWorld(goal);
  }
  bool setDesVelBody_cb(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res) {
    Vec4 goal = Vec4::Zero();
    nh_priv_.param("desVelBody/x", goal(0), 0.0);
    nh_priv_.param("desVelBody/y", goal(1), 0.0);
    nh_priv_.param("desVelBody/z", goal(2), 0.0);
    nh_priv_.param("desVelBody/yaw", goal(3), 0.0);

    return mav_.setDesVelBody(goal);
  }
  bool useRadioForVelocity_cb(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
    static bool use_radio_ = true;
    if (mav_.useRadioForVelocity(use_radio_)) {
      use_radio_ = !use_radio_;
      return true;
    } else
      return false;
  }
  bool hover_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    return mav_.hover();
  }
  bool ehover_cb(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
    return mav_.ehover();
  }
  bool eland_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    return mav_.eland();
  }
  bool estop_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    mav_.estop();
    return true;
  }
  MAV_Services() : nh_(""), nh_priv_("~") {
    // Services
    srv_motors_ =
        nh_.advertiseService("motors", &MAV_Services::motors_cb, this);
    srv_takeoff_ =
        nh_.advertiseService("takeoff", &MAV_Services::takeoff_cb, this);
    srv_goTo_ = nh_.advertiseService("goTo", &MAV_Services::goTo_cb, this);
    srv_setDesVelWorld_ = nh_.advertiseService(
        "setDesVelWorld", &MAV_Services::setDesVelWorld_cb, this);
    srv_setDesVelBody_ = nh_.advertiseService(
        "setDesVelBody", &MAV_Services::setDesVelBody_cb, this);
    srv_useRadioForVelocity_ = nh_.advertiseService(
        "useRadioForVelocity", &MAV_Services::useRadioForVelocity_cb, this);
    srv_hover_ = nh_.advertiseService("hover", &MAV_Services::hover_cb, this);
    srv_ehover_ =
        nh_.advertiseService("ehover", &MAV_Services::ehover_cb, this);
    srv_eland_ = nh_.advertiseService("eland", &MAV_Services::eland_cb, this);
    srv_estop_ = nh_.advertiseService("estop", &MAV_Services::estop_cb, this);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "manager");

  MAV_Services mav_srvs;

  // Let's spin some rotors
  ros::spin();

  return 0;
}
