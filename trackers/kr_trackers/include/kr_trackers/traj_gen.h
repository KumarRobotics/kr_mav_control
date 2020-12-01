#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

class TrajectoryGenerator
{
 public:
  using Vec3f = Eigen::Vector3f;
  using vec_Vec3f = std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>>;

  /**
   * @brief
   *
   * @param continuous_derivative_order The highest derivative that is continous
   * @param minimize_derivative The derivative to minimize
   */
  TrajectoryGenerator(unsigned int continuous_derivative_order, unsigned int minimize_derivative);

  void setInitialConditions(const Vec3f &position, const vec_Vec3f &derivatives);
  void addWaypoint(const Vec3f &position);  // Waypoint is X, Y, Z
  void clearWaypoints(void);
  std::vector<float> computeTimesTrapezoidSpeed(float vel_des, float acc_des) const;
  std::vector<float> computeTimesConstantSpeed(float avg_speed) const;
  bool calculate(const std::vector<float> &waypoint_times_);
  bool getCommand(const float time, Vec3f &pos, Vec3f &vel, Vec3f &acc, Vec3f &jrk) const;

  void calcMaxPerSegment(std::vector<float> &max_vel, std::vector<float> &max_acc, std::vector<float> &max_jrk) const;

  void optimizeWaypointTimes(const float max_vel, const float max_acc, const float max_jrk);

  const std::vector<float> &getWaypointTimes() const;
  float getTotalTime() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const unsigned int N_;
  const unsigned int R_;
  vec_Vec3f waypoints_, initial_derivatives_;
  std::vector<Eigen::MatrixX3f, Eigen::aligned_allocator<Eigen::MatrixX3f>> coefficients_;
  std::vector<float> waypoint_times_;
};
