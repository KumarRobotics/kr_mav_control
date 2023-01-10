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

}  // namespace traj_opt

#endif
