#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <Eigen/StdVector>


#ifndef TRAJECTORY_TYPES_H
#define TRAJECTORY_TYPES_H
typedef double decimal_t; // change to float for single presision

typedef Eigen::Matrix<decimal_t,4,1> Vec4;
typedef Eigen::Matrix<decimal_t,4,4> Mat4;
typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4> > Mat4Vec;
// std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >

#endif