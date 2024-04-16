/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2022-10-24 18:37
#
# Filename: camera_model.hpp
#
# Description:
#
************************************************/

#ifndef _CAMERA_MODEL_HPP_
#define _CAMERA_MODEL_HPP_

#include <boost/array.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

namespace ZJL {

class CameraModel {
public:
  CameraModel(double fx, double fy, double cx, double cy, int width, int height)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width), height_(height) {}
  CameraModel() = default;

  // proj2Image
  // pc - 3d point in camera coordinate system;
  // u(row,col) - pixel coordinates in image;
  bool proj2Image(const Eigen::Vector3d &pc, Eigen::Vector2d &u) const {
    const double fx = fx_;
    const double fy = fy_;
    const double cx = cx_;
    const double cy = cy_;
    const int height = height_;
    const int width = width_;

    const double x = pc(0), y = pc(1), z = pc(2);
    if (fabs(z) < 1e-3) {
      return false;
    }

    u[1] = fx * pc[0] / pc[2] + cx; // cols
    u[0] = fy * pc[1] / pc[2] + cy; // rows

    if (u(0) >= 0 && u(0) < height && u(1) >= 0 && u(1) < width) {
      return true;
    } else {
      return false;
    }
  }

  double getFocal() const { return fx_; }
  double getCenterx() const { return cx_; }
  double getCentery() const { return cy_; }

private:
  double fx_, fy_, cx_, cy_;
  int width_, height_;
};
} // namespace ZJL

#endif
