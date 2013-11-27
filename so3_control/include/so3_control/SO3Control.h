#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

class SO3Control
{
 public:
  SO3Control();

  void setMass(const float mass);
  void setGravity(const float g);
  void setPosition(const Eigen::Vector3f &position);
  void setVelocity(const Eigen::Vector3f &velocity);
  void setMaxIntegral(const float max_integral);
  void resetIntegrals(void);

  void calculateControl(const Eigen::Vector3f &des_pos,
                        const Eigen::Vector3f &des_vel,
                        const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk,
                        const float des_yaw,
                        const float des_yaw_dot,
                        const Eigen::Vector3f &kx,
                        const Eigen::Vector3f &kv,
                        const Eigen::Vector3f &ki);

  const Eigen::Vector3f &getComputedForce(void);
  const Eigen::Quaternionf &getComputedOrientation(void);
  const Eigen::Vector3f &getComputedAngularVelocity(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  float mass_;
  float g_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;
  float max_pos_int_;

  // Outputs of the controller
  Eigen::Vector3f force_;
  Eigen::Quaternionf orientation_;
  Eigen::Vector3f angular_velocity_;
  Eigen::Vector3f pos_int_;
};

#endif
