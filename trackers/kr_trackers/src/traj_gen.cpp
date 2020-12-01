#include <kr_trackers/traj_gen.h>

#include <Eigen/LU>

TrajectoryGenerator::TrajectoryGenerator(unsigned int continuous_derivative_order, unsigned int minimize_derivative)
    : N_(2 * (continuous_derivative_order + 1)), R_(minimize_derivative)
{
}

void TrajectoryGenerator::setInitialConditions(const Vec3f &pos, const vec_Vec3f &derivatives)
{
  clearWaypoints();
  waypoints_.push_back(pos);

  initial_derivatives_.clear();
  initial_derivatives_.resize(N_ / 2);
  for(size_t i = 0; i < std::min(initial_derivatives_.size(), derivatives.size()); ++i)
  {
    initial_derivatives_[i] = derivatives[i];
  }
}

void TrajectoryGenerator::addWaypoint(const Vec3f &x)
{
  if(waypoints_.empty())
    setInitialConditions(x, vec_Vec3f());
  else
    waypoints_.push_back(x);
}

void TrajectoryGenerator::clearWaypoints(void)
{
  waypoints_.clear();
  coefficients_.clear();
  waypoint_times_.clear();
  initial_derivatives_.clear();
}

std::vector<float> TrajectoryGenerator::computeTimesTrapezoidSpeed(float v_des, float a_des) const
{
  std::vector<float> waypoint_times;
  waypoint_times.reserve(waypoints_.size());

  waypoint_times.push_back(0);  // First waypoint, t = 0

  if(waypoints_.size() < 2)
    return waypoint_times;

  const Vec3f &initial_vel_ = initial_derivatives_[0];
  const int initial_vel_sign = (initial_vel_.dot(waypoints_[1] - waypoints_[0]) >= 0) ? 1 : -1;
  const float v_initial = initial_vel_sign * initial_vel_.norm();

  std::vector<float> accumulated_dist;
  accumulated_dist.reserve(waypoints_.size());
  accumulated_dist.push_back(0);
  for(unsigned int i = 1; i < waypoints_.size(); ++i)
  {
    const float dist = (waypoints_[i] - waypoints_[i - 1]).norm();
    accumulated_dist.push_back(accumulated_dist[i - 1] + dist);
  }
  const float total_dist = accumulated_dist.back();

  float d_accel = std::abs(v_des * v_des - v_initial * v_initial) / 2 / a_des;
  float d_decel = v_des * v_des / 2 / a_des;
  float d_constant = (total_dist - (d_accel + d_decel));
  if(d_accel + d_decel > total_dist)
  {
    d_constant = 0;
    d_accel = total_dist / 2 - v_initial * v_initial / 4 / a_des;
    d_decel = total_dist - d_accel;
  }
  const float t_accel = std::sqrt(v_initial * v_initial + 2 * a_des * d_accel) / a_des;
  const float t_constant = d_constant / v_des;
  const float t_decel = std::sqrt(2 * a_des * d_decel) / a_des;

  for(unsigned int i = 1; i < waypoints_.size(); ++i)
  {
    if(accumulated_dist[i] <= d_accel)  // Accel
    {
      waypoint_times.push_back((std::sqrt(v_initial * v_initial + 2 * a_des * accumulated_dist[i]) - v_initial) /
                               a_des);
    }
    else if(accumulated_dist[i] <= d_accel + d_constant)  // Constant
    {
      waypoint_times.push_back(t_accel + (accumulated_dist[i] - d_accel) / v_des);
    }
    else  // Decel
    {
      waypoint_times.push_back(t_accel + t_constant + t_decel -
                               std::sqrt(2 * (total_dist - accumulated_dist[i]) / a_des));
    }
  }

  return waypoint_times;
}

std::vector<float> TrajectoryGenerator::computeTimesConstantSpeed(float avg_speed) const
{
  std::vector<float> waypoint_times;
  waypoint_times.reserve(waypoints_.size());

  waypoint_times.push_back(0);  // First waypoint, t = 0

  for(unsigned int i = 1; i < waypoints_.size(); ++i)
  {
    waypoint_times.push_back(waypoint_times[i - 1] + std::sqrt((waypoints_[i] - waypoints_[i - 1]).norm()) / avg_speed);
  }
  return waypoint_times;
}

const std::vector<float> &TrajectoryGenerator::getWaypointTimes() const
{
  return waypoint_times_;
}

float TrajectoryGenerator::getTotalTime() const
{
  return waypoint_times_.back();
}

// From https://stackoverflow.com/a/33454406
template <typename T>
T powInt(T x, unsigned int n)
{
  if(n == 0)
    return T{1};

  auto y = T{1};
  while(n > 1)
  {
    if(n % 2 == 1)
      y *= x;
    x *= x;
    n /= 2;
  }
  return x * y;
}

bool TrajectoryGenerator::calculate(const std::vector<float> &waypoint_times)
{
  if(waypoints_.size() < 2)
    return false;

  if(waypoint_times.size() != waypoints_.size())
  {
    printf("waypoint_times.size() != waypoints_.size()\n");
    return false;
  }

  const unsigned int num_waypoints = waypoints_.size();
  const unsigned int num_segments = num_waypoints - 1;
  // printf("num_segments: %d\n", num_segments);

  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(num_segments * N_, num_segments * N_);  // Linear constraints
  Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(num_segments * N_, num_segments * N_);  // Quadratic cost matrix
  for(unsigned int i = 0; i < num_segments; i++)
  {
    float seg_time = waypoint_times[i + 1] - waypoint_times[i];
    for(unsigned int n = 0; n < N_; n++)
    {
      // A_0
      if(n < N_ / 2)
      {
        int val = 1;
        for(unsigned int m = 0; m < n; m++)
          val *= (n - m);
        A(i * N_ + n, i * N_ + n) = val;
      }
      // A_T
      for(unsigned int r = 0; r < N_ / 2; r++)
      {
        if(r <= n)
        {
          int val = 1;
          for(unsigned int m = 0; m < r; m++)
            val *= (n - m);
          A(i * N_ + N_ / 2 + r, i * N_ + n) = val * powInt(seg_time, n - r);
        }
      }
      // Q
      for(unsigned int r = 0; r < N_; r++)
      {
        if(r >= R_ && n >= R_)
        {
          int val = 1;
          for(unsigned int m = 0; m < R_; m++)
            val *= (r - m) * (n - m);
          Q(i * N_ + r, i * N_ + n) = 2 * val * powInt(seg_time, r + n - 2 * R_ + 1) / (r + n - 2 * R_ + 1);
        }
      }
    }
  }
  const unsigned int num_fixed_derivatives = num_waypoints - 2 + N_;
  const unsigned int num_free_derivatives = num_waypoints * N_ / 2 - num_fixed_derivatives;
  // printf("num_fixed_derivatives: %d, num_free_derivatives: %d\n",
  //        num_fixed_derivatives, num_free_derivatives);

  // M
  Eigen::MatrixXf M = Eigen::MatrixXf::Zero(num_segments * N_, num_waypoints * N_ / 2);
  M.block(0, 0, N_ / 2, N_ / 2) = Eigen::MatrixXf::Identity(N_ / 2, N_ / 2);
  M.block(num_segments * N_ - N_ / 2, N_ / 2 + num_waypoints - 2, N_ / 2, N_ / 2) =
      Eigen::MatrixXf::Identity(N_ / 2, N_ / 2);
  for(unsigned int i = 0; i < num_waypoints - 2; i++)
  {
    M((2 * i + 1) * N_ / 2, N_ / 2 + i) = 1;
    M((2 * i + 2) * N_ / 2, N_ / 2 + i) = 1;
    for(unsigned int j = 1; j < N_ / 2; j++)
    {
      M((2 * i + 1) * N_ / 2 + j, num_fixed_derivatives - 1 + i * (N_ / 2 - 1) + j) = 1;
      M((2 * i + 2) * N_ / 2 + j, num_fixed_derivatives - 1 + i * (N_ / 2 - 1) + j) = 1;
    }
  }
  // Eigen::MatrixXf A_inv = A.inverse();
  Eigen::MatrixXf A_inv_M = A.partialPivLu().solve(M);
  Eigen::MatrixXf R = A_inv_M.transpose() * Q * A_inv_M;
#if 0
  //std::cout << "A:\n" << A << std::endl;
  std::cout << "Q:\n" << Q << std::endl;
  //std::cout << "M:\n" << M << std::endl;
  //std::cout << "A_inv_M:\n" << A_inv_M << std::endl;
  std::cout << "R:\n" << R << std::endl;
#endif
  Eigen::MatrixXf Rpp =
      R.block(num_fixed_derivatives, num_fixed_derivatives, num_free_derivatives, num_free_derivatives);
  Eigen::MatrixXf Rpf = R.block(num_fixed_derivatives, 0, num_free_derivatives, num_fixed_derivatives);

  // Fixed derivatives
  Eigen::MatrixX3f Df = Eigen::MatrixX3f(num_fixed_derivatives, 3);
  // First point
  Df.row(0) = waypoints_[0].transpose();
  for(unsigned int i = 1; i < N_ / 2; i++)
  {
    Df.row(i) = initial_derivatives_[i - 1].transpose();
  }
  // Middle waypoints
  for(unsigned int i = 1; i < num_waypoints - 1; i++)
  {
    Df.row((N_ / 2) - 1 + i) = waypoints_[i].transpose();
  }
  // End point
  Df.row(N_ / 2 + (num_waypoints - 2)) = waypoints_[num_waypoints - 1].transpose();
  for(unsigned int i = 1; i < N_ / 2; i++)
  {
    Df.row((N_ / 2) + (num_waypoints - 2) + i) = Vec3f::Zero().transpose();
  }
  // std::cout << "Df:\n" << Df << std::endl;
  Eigen::MatrixX3f D = Eigen::MatrixX3f(num_waypoints * N_ / 2, 3);
  D.topRows(num_fixed_derivatives) = Df;
  if(num_waypoints > 2 && num_free_derivatives > 0)
  {
    Eigen::MatrixX3f Dp = -Rpp.partialPivLu().solve(Rpf * Df);
    // std::cout << "Dp:\n" << Dp << std::endl;
    D.bottomRows(num_free_derivatives) = Dp;
  }
  Eigen::MatrixX3f d = M * D;
  // std::cout << "d:\n" << d << std::endl;
  coefficients_.clear();
  for(unsigned int i = 0; i < num_segments; i++)
  {
    const Eigen::MatrixX3f p = A.block(i * N_, i * N_, N_, N_).partialPivLu().solve(d.block(i * N_, 0, N_, 3));
    // std::cout << "p:\n" << p << std::endl;
    coefficients_.push_back(p);
  }
  waypoint_times_ = waypoint_times;
  return true;
}

bool TrajectoryGenerator::getCommand(const float time, Vec3f &pos, Vec3f &vel, Vec3f &acc, Vec3f &jrk) const
{
  if(time < 0)
    return false;

  int cur_idx = -1;
  for(unsigned int i = 1; i < waypoint_times_.size(); i++)
  {
    if(time <= waypoint_times_[i])
    {
      cur_idx = i - 1;
      break;
    }
  }
  if(cur_idx == -1)
    return false;

  const float t_traj = time - waypoint_times_[cur_idx];
  const Eigen::MatrixX3f &p = coefficients_[cur_idx];
  pos = Vec3f::Zero();
  for(unsigned int i = 0; i < p.rows(); i++)
    pos += p.row(i).transpose() * powInt(t_traj, i);

  vel = Vec3f::Zero();
  for(unsigned int i = 1; i < p.rows(); i++)
    vel += p.row(i).transpose() * (i * powInt(t_traj, i - 1));

  acc = Vec3f::Zero();
  for(unsigned int i = 2; i < p.rows(); i++)
    acc += p.row(i).transpose() * (i * (i - 1) * powInt(t_traj, i - 2));

  jrk = Vec3f::Zero();
  for(unsigned int i = 3; i < p.rows(); i++)
    jrk += p.row(i).transpose() * (i * (i - 1) * (i - 2) * powInt(t_traj, i - 3));

  return true;
}

void TrajectoryGenerator::calcMaxPerSegment(std::vector<float> &max_vel, std::vector<float> &max_acc,
                                            std::vector<float> &max_jrk) const
{
  const unsigned int num_samples_per_seg = 10;
  for(unsigned int seg_idx = 0; seg_idx < waypoint_times_.size() - 1; ++seg_idx)
  {
    const float seg_start_time = waypoint_times_[seg_idx];
    const float seg_end_time = waypoint_times_[seg_idx + 1];
    float seg_duration = seg_end_time - seg_start_time;
    const float dt = seg_duration / num_samples_per_seg;

    float seg_max_vel = 0, seg_max_acc = 0, seg_max_jrk = 0;
    for(unsigned int sample_idx = 0; sample_idx < num_samples_per_seg; ++sample_idx)
    {
      const float t_traj = sample_idx * dt;
      const Eigen::MatrixX3f &p = coefficients_[seg_idx];
      Vec3f vel = Vec3f::Zero();
      for(unsigned int i = 1; i < p.rows(); i++)
        vel += p.row(i).transpose() * (i * powInt(t_traj, i - 1));
      if(vel.norm() > seg_max_vel)
        seg_max_vel = vel.norm();

      Vec3f acc = Vec3f::Zero();
      for(unsigned int i = 2; i < p.rows(); i++)
        acc += p.row(i).transpose() * (i * (i - 1) * powInt(t_traj, i - 2));
      if(acc.norm() > seg_max_acc)
        seg_max_acc = acc.norm();

      Vec3f jrk = Vec3f::Zero();
      for(unsigned int i = 3; i < p.rows(); i++)
        jrk += p.row(i).transpose() * (i * (i - 1) * (i - 2) * powInt(t_traj, i - 3));
      if(jrk.norm() > seg_max_jrk)
        seg_max_jrk = jrk.norm();
    }
    // printf("seg idx: %d\n", seg_idx);
    // printf("max_vel: %f\n", seg_max_vel);
    // printf("max_acc: %f\n", seg_max_acc);
    // printf("max_jrk: %f\n", seg_max_jrk);
    max_vel.push_back(seg_max_vel);
    max_acc.push_back(seg_max_acc);
    max_jrk.push_back(seg_max_jrk);
  }
  return;
}

void TrajectoryGenerator::optimizeWaypointTimes(const float max_vel, const float max_acc, const float max_jrk)
{
  std::vector<float> segment_times;
  for(unsigned int i = 0; i < waypoint_times_.size() - 1; ++i)
    segment_times.push_back(waypoint_times_[i + 1] - waypoint_times_[i]);

  std::vector<float> best_segment_times = segment_times;
  float best_cost = std::numeric_limits<float>::max();

  for(int traj_iter = 0; traj_iter < 20; ++traj_iter)
  {
    // std::cout << std::string(20, '=') << " " << traj_iter << " "
    //           << std::string(20, '=') << "\n";

    for(unsigned int i = 1; i < waypoint_times_.size(); ++i)
    {
      waypoint_times_[i] = waypoint_times_[i - 1] + segment_times[i - 1];
    }
    calculate(waypoint_times_);

    std::vector<float> seg_max_vel, seg_max_acc, seg_max_jrk;
    calcMaxPerSegment(seg_max_vel, seg_max_acc, seg_max_jrk);

    const float vel_cost = 1.0f, acc_cost = 0.1f, jrk_cost = 0.01f;
    float cost = 0;

    const auto rectified_square = [](float x) { return (x > 0) ? x * x : 0; };

    const std::vector<float> prev_segment_times = segment_times;
    bool done = true;
    for(unsigned int i = 0; i < seg_max_vel.size(); ++i)
    {
      cost += vel_cost * rectified_square(seg_max_vel[i] - max_vel) +
              acc_cost * rectified_square(seg_max_acc[i] - max_acc) +
              jrk_cost * rectified_square(seg_max_jrk[i] - max_jrk);

      if(seg_max_vel[i] > max_vel || seg_max_acc[i] > max_acc || seg_max_jrk[i] > max_jrk)
      {
        segment_times[i] *= 1.035f;
        done = false;
      }
      else if(seg_max_vel[i] < 0.9f * max_vel && seg_max_acc[i] < 0.9f * max_acc && seg_max_jrk[i] < 0.9f * max_jrk)
      {
        segment_times[i] *= 0.966f;
        done = false;
      }
    }

    // std::cout << "cost: " << cost << "\n";
    // If no solution found, keep the "best" one (with lowest violations)
    if(cost <= best_cost)
    {
      best_segment_times = prev_segment_times;
      best_cost = cost;
    }
    if(done)
    {
      // std::cout << "Early exit! num iter: " << traj_iter << "\n";
      break;
    }
  }
}
