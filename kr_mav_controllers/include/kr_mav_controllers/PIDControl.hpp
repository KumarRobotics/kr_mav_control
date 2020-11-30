#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Eigen/Geometry>

class PIDControl
{
 public:
  PIDControl();

  void setMass(const float mass);
  void setGravity(const float g);
  void setPosition(const Eigen::Vector3f &position);
  void setVelocity(const Eigen::Vector3f &velocity);
  void setYaw(const float current_yaw);
  void setMaxIntegral(const float max_integral);
  void resetIntegrals(void);

  void calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const float des_yaw, const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                        const Eigen::Vector3f &ki, const float ki_yaw);

  const Eigen::Vector4f &getControls(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  float mass_;
  float g_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;
  float current_yaw_;
  Eigen::Vector3f pos_int_;
  float yaw_int_;
  float max_pos_int_;

  // Outputs of the controller
  Eigen::Vector4f trpy_;
};

#endif
