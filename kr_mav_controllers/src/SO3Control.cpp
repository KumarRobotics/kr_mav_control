#include "kr_mav_controllers/SO3Control.h"

#include <ros/console.h>
#include <tf/transform_datatypes.h>

SO3Control::SO3Control()
    : mass_(0.5),
      g_(9.81),
      max_pos_int_(0.5),
      max_pos_int_b_(0.5),
      current_orientation_(Eigen::Quaternionf::Identity()),
      cos_max_tilt_angle_(-1.0)
{
}

void SO3Control::setMass(const float mass)
{
  mass_ = mass;
}

void SO3Control::setGravity(const float g)
{
  g_ = g;
}

void SO3Control::setPosition(const Eigen::Vector3f &position)
{
  pos_ = position;
}

void SO3Control::setVelocity(const Eigen::Vector3f &velocity)
{
  vel_ = velocity;
}

void SO3Control::setMaxIntegral(const float max_integral)
{
  max_pos_int_ = max_integral;
}

void SO3Control::setMaxIntegralBody(const float max_integral_b)
{
  max_pos_int_b_ = max_integral_b;
}

void SO3Control::setCurrentOrientation(const Eigen::Quaternionf &current_orientation)
{
  current_orientation_ = current_orientation;
}

void SO3Control::setMaxTiltAngle(const float max_tilt_angle)
{
  if(max_tilt_angle > 0.0f && max_tilt_angle <= static_cast<float>(M_PI))
    cos_max_tilt_angle_ = std::cos(max_tilt_angle);
}

void SO3Control::calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel,
                                  const Eigen::Vector3f &des_acc, const Eigen::Vector3f &des_jerk, const float des_yaw,
                                  const float des_yaw_dot, const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                  const Eigen::Vector3f &ki, const Eigen::Vector3f &ki_b)
{
  const Eigen::Vector3f e_pos = des_pos - pos_;
  const Eigen::Vector3f e_vel = des_vel - vel_;

  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_(i) += ki(i) * e_pos(i);

    // Limit integral term
    if(pos_int_(i) > max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) < -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }
  // ROS_DEBUG_THROTTLE(2, "Integrated world disturbance compensation [N]: {x: %2.2f, y: %2.2f, z: %2.2f}", pos_int_(0),
  // pos_int_(1), pos_int_(2));

  Eigen::Quaternionf q(current_orientation_);
  const Eigen::Vector3f e_pos_b = q.inverse() * e_pos;
  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_b_(i) += ki_b(i) * e_pos_b(i);

    // Limit integral term in the body
    if(pos_int_b_(i) > max_pos_int_b_)
      pos_int_b_(i) = max_pos_int_b_;
    else if(pos_int_b_(i) < -max_pos_int_b_)
      pos_int_b_(i) = -max_pos_int_b_;
  }
  // ROS_DEBUG_THROTTLE(2, "Integrated body disturbance compensation [N]: {x: %2.2f, y: %2.2f, z: %2.2f}",
  // pos_int_b_(0), pos_int_b_(1), pos_int_b_(2));

  const Eigen::Vector3f acc_grav = g_ * Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f acc_control = kx.asDiagonal() * e_pos + kv.asDiagonal() * e_vel + pos_int_ + des_acc;
  Eigen::Vector3f acc_total = acc_control + acc_grav;

  // Check and limit tilt angle
  float lambda = 1.0f;
  if(Eigen::Vector3f::UnitZ().dot(acc_total.normalized()) < cos_max_tilt_angle_)
  {
    const float x = acc_control.x(), y = acc_control.y(), z = acc_control.z();
    const float cot_max_tilt_angle = cos_max_tilt_angle_ / std::sqrt(1 - cos_max_tilt_angle_ * cos_max_tilt_angle_);
    lambda = -g_ / (z - std::sqrt(x * x + y * y) * cot_max_tilt_angle);
    if(lambda > 0 && lambda <= 1)
      acc_total = lambda * acc_control + acc_grav;
  }

  force_.noalias() = mass_ * acc_total;

  // std::cout << "Force: " << force_.transpose() << std::endl;

  Eigen::Vector3f b1c, b2c, b3c;
  const Eigen::Vector3f b2d(-std::sin(des_yaw), std::cos(des_yaw), 0);

  if(force_.norm() > 1e-6f)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3f::UnitZ();

  b1c.noalias() = b2d.cross(b3c).normalized();
  b2c.noalias() = b3c.cross(b1c).normalized();

  const Eigen::Vector3f force_dot =
      mass_ * lambda * (kx.asDiagonal() * e_vel + des_jerk);  // Ignoring kv*e_acc and ki*e_pos terms
  const Eigen::Vector3f b3c_dot = b3c.cross(force_dot / force_.norm()).cross(b3c);
  const Eigen::Vector3f b2d_dot(-std::cos(des_yaw) * des_yaw_dot, -std::sin(des_yaw) * des_yaw_dot, 0);
  const Eigen::Vector3f b1c_dot =
      b1c.cross(((b2d_dot.cross(b3c) + b2d.cross(b3c_dot)) / (b2d.cross(b3c)).norm()).cross(b1c));
  const Eigen::Vector3f b2c_dot = b3c_dot.cross(b1c) + b3c.cross(b1c_dot);

  Eigen::Matrix3f R;
  R << b1c, b2c, b3c;
  orientation_ = Eigen::Quaternionf(R);

  Eigen::Matrix3f R_dot;
  R_dot << b1c_dot, b2c_dot, b3c_dot;

  const Eigen::Matrix3f omega_hat = R.transpose() * R_dot;
  angular_velocity_ = Eigen::Vector3f(omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0));
}

const Eigen::Vector3f &SO3Control::getComputedForce()
{
  return force_;
}

const Eigen::Quaternionf &SO3Control::getComputedOrientation()
{
  return orientation_;
}

const Eigen::Vector3f &SO3Control::getComputedAngularVelocity()
{
  return angular_velocity_;
}

void SO3Control::resetIntegrals()
{
  pos_int_ = Eigen::Vector3f::Zero();
  pos_int_b_ = Eigen::Vector3f::Zero();
}
