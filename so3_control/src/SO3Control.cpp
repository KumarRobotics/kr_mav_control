#include "so3_control/SO3Control.h"

SO3Control::SO3Control()
  : mass_(0.5),
    g_(9.81),
    max_pos_int_(0.5)
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

void SO3Control::calculateControl(const Eigen::Vector3f &des_pos,
                                  const Eigen::Vector3f &des_vel,
                                  const Eigen::Vector3f &des_acc,
                                  const Eigen::Vector3f &des_jerk,
                                  const float des_yaw,
                                  const float des_yaw_dot,
                                  const Eigen::Vector3f &kx,
                                  const Eigen::Vector3f &kv,
                                  const Eigen::Vector3f &ki)
{
  const Eigen::Vector3f e_pos = des_pos - pos_;
  const Eigen::Vector3f e_vel = des_vel - vel_;

  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_(i) += ki(i)*e_pos(i);

    // Limit integral term
    if(pos_int_(i) > max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) < -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }

  //std::cout << "pos_int: " << pos_int_.transpose() << std::endl;

  force_.noalias() = kx.asDiagonal()*e_pos  + kv.asDiagonal()*e_vel + pos_int_ + mass_*g_*Eigen::Vector3f(0, 0, 1) +
      mass_ * des_acc;

  //std::cout << "Force: " << force_.transpose() << std::endl;

  Eigen::Vector3f b1c, b2c, b3c;
  Eigen::Vector3f b2d(-std::sin(des_yaw), std::cos(des_yaw), 0);

  if(force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3f(0, 0, 1);

  b1c.noalias() = b2d.cross(b3c).normalized();
  b2c.noalias() = b3c.cross(b1c).normalized();

  const Eigen::Vector3f force_dot = kx.asDiagonal()*e_vel + mass_*des_jerk; // Ignoring kv*e_acc and ki*e_pos terms
  const Eigen::Vector3f b3c_dot = b3c.cross(force_dot/force_.norm()).cross(b3c);
  const Eigen::Vector3f b2d_dot(-std::cos(des_yaw)*des_yaw_dot, std::sin(des_yaw)*des_yaw_dot, 0);
  const Eigen::Vector3f b1c_dot = b1c.cross(((b2d_dot.cross(b3c)+b2d.cross(b3c_dot))/(b2d.cross(b3c)).norm()).cross(b1c));
  const Eigen::Vector3f b2c_dot = b3c_dot.cross(b1c) + b3c.cross(b1c_dot);

  Eigen::Matrix3f R;
  R << b1c, b2c, b3c;
  orientation_ = Eigen::Quaternionf(R);

  Eigen::Matrix3f R_dot;
  R_dot << b1c_dot, b2c_dot, b3c_dot;

  const Eigen::Matrix3f omega_hat = R.transpose()*R_dot;
  angular_velocity_ = Eigen::Vector3f(omega_hat(2,1), omega_hat(0,2), omega_hat(1,0));

}

const Eigen::Vector3f &SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Quaternionf &SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

const Eigen::Vector3f &SO3Control::getComputedAngularVelocity(void)
{
  return angular_velocity_;
}

void SO3Control::resetIntegrals(void)
{
  pos_int_ = Eigen::Vector3f::Zero();
}
