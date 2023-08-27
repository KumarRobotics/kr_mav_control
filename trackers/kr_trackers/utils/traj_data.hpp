// Copyright 2015 Michael Watterson, University of Pennsylvania
// Revised 2022 by Yuwei
#ifndef TRAJ_DATcoeffsHPP
#define TRAJ_DATcoeffsHPP

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>

namespace traj_opt
{

enum PolyType
{
  STANDARD,  // polynomials
  BEZIER,    // B-spline and bernstein 
  BERNSTEIN,
  ENDPOINT,
  CHEBYSHEV
};

/**
 * @brief 
 * 
 * - STANDARD: polynomial trajectory
 * 
 * p(t) =  C * [1, t, t^2, t^3, t^4, t^5]^T
 * 
 * polynomials are stored with parameterization s \in [0,1]
 * time dt dt is used to evaluate polynomial p(t/dt) for t \in [0,dt]
 *
 * - BEZIER:
 * - BERNSTEIN: 
 * 
 * p(t) = cpts * A * [1, t, t^2, t^3, t^4, t^5]^T
 * cpts: control points  (N+1)x3 
 * A: transformation matrix (N+1) * (N+1)
 * 
 * then C = cpts * A can also represent with general matrix representations
 * 
 * reference:
 * Kaihuai Qin, "General matrix representations for B-splines," 
 * Proceedings Pacific Graphics '98. Sixth Pacific Conference on Computer Graphics and Applications (Cat. No.98EX208), 
 * 1998, pp. 37-43, doi: 10.1109/PCCGA.1998.731996.
 * 
 **/

// one segment of a trajectory, one dimension
template <int dim>
class Piece
{
 private:
  int degree = 5;  // set 5
  float dt;        // dt
  PolyType basis;
  Eigen::MatrixXd ncoeffs;

 public:

  Piece(){}
  ~Piece(){}

  Piece(PolyType ptype, Eigen::MatrixXd ncoeffs_matrix, float dur)
  {
    basis = ptype;
    ncoeffs = ncoeffs_matrix;
    dt = dur;
    degree = ncoeffs.cols() -1;
  }

  Piece(PolyType ptype, Eigen::MatrixXd cpts, float dur, int deg)
  {
    basis = ptype;
    degree = deg;
    dt = dur;
    getNcoeffs(cpts);
  }

  inline double getDur() const{   return dt;}

  inline Eigen::VectorXd getPos(double t)
  {
    // Normalize the time
    t /= dt;
    Eigen::VectorXd pos(dim);
    pos.setZero();
    double tn = 1.0;
    for (int i = 0; i <= degree; i++)
    {
        pos += tn * ncoeffs.col(i);
        tn *= t;
    }
    return pos;
  }

  // Get the velocity at time t in this piece
  inline Eigen::VectorXd getVel(double t)
  {
    t /= dt;
    Eigen::VectorXd vel(dim);
    vel.setZero();
    double tn = 1.0;
    int n = 1;
    for (int i = 1; i <= degree; i++)
    {
        vel += n * tn * ncoeffs.col(i);
        tn *= t;
        n++;
    }
    vel /= dt;
    return vel;
  }

  // Get the acceleration at time t in this piece
  inline Eigen::VectorXd getAcc(double t)
  {
    t /= dt;
    Eigen::VectorXd acc(dim);
    acc.setZero();
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = 2; i <= degree; i++)
    {
        acc += m * n * tn * ncoeffs.col(i);
        tn *= t;
        m++;
        n++;
    }
    acc /= dt * dt;
    return acc;
  }


 private:

  void getNcoeffs(Eigen::MatrixXd& cpts)
  {
    Eigen::MatrixXd A(degree + 1, degree + 1);
    A.setZero();
    switch (basis)
    {
      case BEZIER:
      {
        switch (degree) 
        {
          case 1:
            A <<  1, -1,
                  0, 1;
            break;
          case 2:
          {
            A <<  1, -2, 1, 
                  1, 2, -2, 
                  0, 0, 1;
            A /= 2.0;
            break;
          }
          case 3:
          {
            A <<  1, -3, 3, 1,
                  4, 0, -6, 3,
                  1, 3, 3, -3,
                  0, 0, 0, 1;
            A /= 6.0;
            break;
          }
          case 4:
          {
            A <<  1, -4,   6,  -4,  1,
                  11,-12, -6,  12, -4,
                  11, 12, -6, -12,  6,
                  1,  4,   6,   4, -4,
                  0,  0,   0,   0,  1;
            A /= 24.0;
            break;
          }
        }
        break;
      }
      case BERNSTEIN:
      {
        switch (degree) 
        {
          case 1:
            A <<  1, -1,
                  0, 1;
            break;
          case 2:
            A <<  1, -2, 1, 
                  0, 2, -2, 
                  0, 0, 1;
            break;
          case 3:
            A <<  1, -3, 3, 1,
                  0, 3, -6, 3,
                  0, 0, 3, -3,
                  0, 0, 0, 1;
            break;
          case 4:
            A <<  1, -4,   6,  -4,  1,
                  0,  4, -12,  12, -4,
                  0,  0,   6, -12,  6,
                  0,  0,   0,   4, -4,
                  0,  0,   0,   0,  1;
            break;
        }
        break;
      }
    }
    ncoeffs = cpts.transpose() * A;

  }

};


/**
 * @brief trajectory data
 * @tparam dim = 1-4
 */
template <int dim>
class Trajectory
{
 private:
  std::vector<Piece<dim>> seg_pieces;
  int seg_num;
  double dttotal;

 public:
  Trajectory(){}
  ~Trajectory(){}

  Trajectory(std::vector<Piece<dim>> segs, double dur)
  {
    seg_pieces = segs;
    dttotal = dur;
    seg_num = seg_pieces.size();
  }

  inline bool isValid() const
  {
    if(seg_num <= 0){ return false;}
    return true;
  }

  // Find the piece at which the time t is located
  // The index is returned and the offset in t is removed
  inline int locatePieceIdx(double &t) const
  {
    int idx;
    double dur;
    for(idx = 0; idx < seg_num && t > (dur = seg_pieces[idx].getDur()); idx++)
    {
      t -= dur;
    }
    if(idx == seg_num)
    {
      idx--;
      t += seg_pieces[idx].getDur();
    }
    return idx;
  }

  inline Eigen::VectorXd getPos(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getPos(t);
  }

  inline Eigen::VectorXd getVel(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getVel(t);
  }

  inline Eigen::VectorXd getAcc(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getAcc(t);
  }
};

typedef Trajectory<1> Trajectory1D;
/// Trajectory in 2D
typedef Trajectory<2> Trajectory2D;
/// Trajectory in 3D
typedef Trajectory<3> Trajectory3D;
/// Trajectory in 3D with yaw
typedef Trajectory<4> Trajectory4D;


class DiscreteStates
{
  private:
    int N = 20;
    double dt;

    std::vector<Eigen::VectorXd> states;  // p v a
    bool is_linear_cut = false;
  
  public:

    DiscreteStates(){}
    DiscreteStates(const double interval, 
                   const int num,
                   std::vector<Eigen::VectorXd> &discrete_states)
        : dt(interval), N(num), states(discrete_states)
    {

    }
    ~DiscreteStates() = default;

    inline void useLinearCut()
    {
      is_linear_cut = true;
    }

    inline Eigen::VectorXd getState(double t)
    {

      int index = std::floor(t / dt);

      Eigen::VectorXd x1  = states[index];
      Eigen::VectorXd x2 =  states[index+1];

      double tau =  t - index * dt;


      if(is_linear_cut)
      {
        return x1 + tau / dt * (x2 - x1);
      }

      Eigen::MatrixXd psi = Q(tau) * (Phi(dt - tau).transpose()) * QInverse(dt);
      Eigen::MatrixXd lambda = Phi(tau) - psi * Phi(dt);

      return lambda * x1 + psi * x2;
    }


    
    inline Eigen::VectorXd getPreState(double t)
    {

      int index = std::floor(t / dt);

      return states[index];
    }


    inline Eigen::VectorXd getNextState(double t)
    {

      int index = std::floor(t / dt);

      return states[index+1];
    }


    inline Eigen::Vector3d getNextPos(double t)
    {

      int index = std::floor(t / dt);

      return states[index+1].head(3);
    }



  private:

    //state transit matrix  9 * 9 
    static inline Eigen::MatrixXd Phi(const double tau)
    {
      Eigen::MatrixXd phi = Eigen::Matrix<double, 9, 9>::Identity();
      
      for (int i = 0; i < 6; ++i)
        phi(i, i + 3) = tau;

      for (int i = 0; i < 3; ++i)
        phi(i, i + 6) = 0.5 * tau * tau;
      //std::cout << "   phi  " << phi << std::endl;
      return phi;
    }


    static inline Eigen::MatrixXd QInverse(const double tau) 
    {
      Eigen::MatrixXd q_inv(9, 9);
      q_inv.setZero();
      std::array<double, 5> tau_powers;
      const double tau_inv = 1.0 / tau;

      Eigen::Matrix3d qc_inv = Eigen::Matrix3d::Identity();

      
      tau_powers[0] = tau_inv;
      for (int i = 1; i < 5; ++i) tau_powers[i] = tau_inv * tau_powers[i - 1];

      q_inv.block(0, 0, 3, 3) = 720 * tau_powers[4] * qc_inv;
      q_inv.block(0, 3, 3, 3) = -360 * tau_powers[3]* qc_inv;
      q_inv.block(0, 6, 3, 3) = 60 * tau_powers[2]* qc_inv;
      q_inv.block(3, 0, 3, 3) = q_inv(0, 1)* qc_inv;
      q_inv.block(3, 3, 3, 3) = 192 * tau_powers[2]* qc_inv;
      q_inv.block(3, 6, 3, 3) = -36 * tau_powers[1]* qc_inv;
      q_inv.block(6, 0, 3, 3) = q_inv(0, 2)* qc_inv;
      q_inv.block(6, 3, 3, 3) = q_inv(1, 2)* qc_inv;
      q_inv.block(6, 6, 3, 3) = 9 * tau_powers[0]* qc_inv;

      return q_inv;
    }

    static inline Eigen::MatrixXd Q(const double tau) 
    {

      Eigen::MatrixXd q(9, 9);
      q.setZero();
      std::array<double, 5> tau_powers;
      constexpr double one_third = 1.0 / 3.0;
      constexpr double one_sixth = 1.0 / 6.0;

      Eigen::Matrix3d qc = Eigen::Matrix3d::Identity();


      tau_powers[0] = tau;
      for (int i = 1; i < 5; ++i) tau_powers[i] = tau * tau_powers[i - 1];

      q.block(0, 0, 3, 3) = 0.05 * tau_powers[4] * qc;
      q.block(0, 3, 3, 3) = 0.125 * tau_powers[3] * qc;
      q.block(0, 6, 3, 3) = one_sixth * tau_powers[2] * qc;
      q.block(3, 0, 3, 3) = q(0, 1) * qc;
      q.block(3, 3, 3, 3) = one_third * tau_powers[2] * qc;
      q.block(3, 6, 3, 3) = 0.5 * tau_powers[1] * qc;
      q.block(6, 0, 3, 3) = q(0, 2) * qc;
      q.block(6, 3, 3, 3) = q(1, 2) * qc;
      q.block(6, 6, 3, 3) = tau_powers[0] * qc;

      return q;
    }



};





}  // namespace traj_opt

#endif
