#include <mav_manager/manager.h>
#include <std_srvs/Empty.h>
#include <mav_manager/Vec4.h>

// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Services {
 public:
  // Services
  ros::ServiceServer srv_motors_, srv_takeoff_, srv_goTo_, srv_setDesVelInWorldFrame_,
      srv_setDesVelInBodyFrame_, srv_hover_, srv_ehover_,
      srv_eland_, srv_estop_;

  // Let's make an MAV
  MAVManager mav_;

  bool motors_cb(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
    static bool motor_toggle_ = true;
    mav_.set_motors(motor_toggle_);
    motor_toggle_ = !motor_toggle_;
    return true;
  }
  bool takeoff_cb(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res) {
    return mav_.takeoff();
  }
  bool goTo_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

    double x, y, z, yaw;
    nh_priv_.param("goTo/x", x, 0.0);
    nh_priv_.param("goTo/y", y, 0.0);
    nh_priv_.param("goTo/z", z, 0.0);
    nh_priv_.param("goTo/yaw", yaw, 0.0);

    return mav_.goTo(x, y, z, yaw);
  }
  bool setDesVelInWorldFrame_cb(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res) {

    double x, y, z, yaw;
    nh_priv_.param("desVelWorld/x", x, 0.0);
    nh_priv_.param("desVelWorld/y", y, 0.0);
    nh_priv_.param("desVelWorld/z", z, 0.0);
    nh_priv_.param("desVelWorld/yaw", yaw, 0.0);

    return mav_.setDesVelInWorldFrame(x, y, z, yaw);
  }
  bool setDesVelInBodyFrame_cb(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res) {

    double x, y, z, yaw;
    nh_priv_.param("desVelBody/x", x, 0.0);
    nh_priv_.param("desVelBody/y", y, 0.0);
    nh_priv_.param("desVelBody/z", z, 0.0);
    nh_priv_.param("desVelBody/yaw", yaw, 0.0);

    return mav_.setDesVelInBodyFrame(x, y, z, yaw);
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
    srv_setDesVelInWorldFrame_ = nh_.advertiseService(
        "setDesVelInWorldFrame", &MAV_Services::setDesVelInWorldFrame_cb, this);
    srv_setDesVelInBodyFrame_ = nh_.advertiseService(
        "setDesVelInBodyFrame", &MAV_Services::setDesVelInBodyFrame_cb, this);
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
