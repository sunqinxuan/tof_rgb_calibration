/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-10-28 09:19
#
# Filename:		registration.hpp
#
# Description:
#
************************************************/

#ifndef _REGISTRATION_HPP_
#define _REGISTRATION_HPP_

//#include "models/line_feature_extraction/line_feature_extraction_rg.hpp"
//#include "models/registration/registration_interface.hpp"

//#include "sensor_data/cloud_data.hpp"
//#include "sensor_data/line_feature.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <ceres/ceres.h>
#include <cmath>
#include <fstream>
#include <functional>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/bfgs.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "camera_model.hpp"

namespace ZJL {
class CannyRegistration {
public:
  using Matrix6f = Eigen::Matrix<float, 6, 6>;
  using Vector6f = Eigen::Matrix<float, 6, 1>;
  using KdTree = pcl::KdTreeFLANN<pcl::PointXY>;
  using KdTreePtr = KdTree::Ptr;
  using PixelCloud = pcl::PointCloud<pcl::PointXY>;
  using PixelCloudPtr = PixelCloud::Ptr;
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
  using PointCloudPtr = PointCloud::Ptr;
  using VecMat =
      std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>>;
  using VecMatPtr = std::shared_ptr<VecMat>;

  CannyRegistration(const YAML::Node &node);

  bool setInputTarget(const PixelCloudPtr &input_edge_target);
  bool alignScans(const PixelCloudPtr &input_edge_source,
                  const PointCloudPtr &input_point_cloud,
                  const CameraModel &cam, Eigen::Isometry3d &T_tgt_src);

  double getReprojError(const PixelCloudPtr &input_edge_source,
                        const PointCloudPtr &input_point_cloud,
                        const CameraModel &cam, Eigen::Isometry3d &T_tgt_src,
                        double max_dist);

  // debug
  void drawLines(cv::Mat &image, const CameraModel &cam);

private:
  double max_corr_dist_ = 100;
  int num_neighbors_cov_ = 20;
  // double sgicp_epsilon_ = 0.001;

  bool converged_ = false;
  int max_iterations_ = 200;
  double thres_trs_ = 1e-6;   //(3e-4 * 3e-4)  // 0.0003 meters
  double thres_rot_ = 0.9999; //(0.99999)         // 0.256 degrees

  PixelCloudPtr input_edge_tgt_;
  PixelCloudPtr input_edge_src_;
  PointCloudPtr input_point_cloud_;

  CameraModel camera_model_;

  KdTreePtr kdtree_;

  std::vector<int> indices_src_, indices_tgt_;
  VecMatPtr covariances_tgt_;

  Eigen::Isometry3d transformation_;

  ceres::LinearSolverType linear_solver_type_ = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::TrustRegionStrategyType trust_region_strategy_type_ =
      ceres::LEVENBERG_MARQUARDT;
  unsigned int max_num_iterations_ = 10;
  bool minimizer_progress_to_stdout_ = false;

  // const int SEMANTIC_NUMS = 7; //语义种类数量(加上地面类)
  //
  // int max_inner_iterations_ = 20;
  //
  //// threshold for convergence of ICP iteration;
  // float rotation_epsilon_ = 2e-4;
  // float transformation_epsilon_ = 5e-4;
  //
  //// epsilon in the adjusted covariance matrix;
  //// number of neighbors for PCA;
  //
  //// threshold for point correspondences;
  // float max_dist_;
  // float min_dist_;
  // float correspondences_cur_mse_ = 10000.0;
  // float correspondences_prev_mse_ = 0.0;
  // float correspondences_cur_max_;
  //

  // float mse_threshold_relative_; //(0.00001)     // 0.001% of the previous
  // MSE
  //                               //(relative error)
  // float mse_threshold_absolute_; //(1e-12)       // MSE (absolute error)

private:
  // int GetCorrespondence(const pcl::PointCloud<pcl::PointXY>::Ptr
  // &source_cloud,
  //                      const KdTreePtr &target_kdtree,
  //                      const VecLine &source_lines,
  //                      std::vector<int> &source_indices,
  //                      std::vector<int> &target_indices);
  //
  bool isConverged(const Eigen::Isometry3d &delta_transformation);
  bool computeCovariance(const PixelCloudPtr &cloud,
                         const std::vector<int> &cloud_indices,
                         const KdTreePtr &tree, VecMatPtr &covs);
  // bool UpdateCorrMaxDist(const float &mean_dist, const float &max_dist,
  //                       const size_t &pc_size, const size_t &inline_size);
  //
  // bool CloudClassify(const CloudData::CLOUD_PTR &input, VecCloud &group,
  //                   std::vector<bool> &empty,
  //                   std::vector<VecLine> &group_lines,
  //                   std::vector<std::vector<int>> &group_line_indices);
  //
  // float Point2LineDistance(const CloudData::POINTXYZI &pt,
  //                         const LineFeature &line);
  // pcl::PointXY TransformPointXY(const Eigen::Isometry2f &trans,
  //                              const pcl::PointXY &point);
  // pcl::PointXY TransformPointXY(const Eigen::Isometry2f &trans,
  //                              const Eigen::Vector2f &point);
  //
  // Eigen::Isometry2f previous_transformation_;
  //
  // VecCloud input_target_group_;
  // VecKdTreePtr input_target_group_kdtree_;
  // std::vector<bool> input_target_group_empty_;
  // std::vector<VecLine> input_target_group_lines_;
  // std::vector<std::vector<int>> input_target_group_lines_indices_;
  //
  // VecCloud input_source_group_;
  // std::vector<bool> input_source_group_empty_;
  // std::vector<VecLine> input_source_group_lines_;
  // std::vector<std::vector<int>> input_source_group_lines_indices_;
  //
  // std::shared_ptr<LineFeatureExtractionInterface> line_extract_ptr_;
  //
  // std::vector<std::vector<int>> source_indices_group_;
  // std::vector<std::vector<int>> target_indices_group_;

  bool BuildCeresProblem(ceres::Problem *problem,
                         const std::vector<int> &indices_src,
                         const std::vector<int> &indices_tgt,
                         Eigen::Vector3d &trns, Eigen::Quaterniond &quat);
  bool SolveCeresProblem(ceres::Problem *problem);

  // template <typename T> static Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw)
  // {
  //  const T cos_yaw = ceres::cos(yaw);
  //  const T sin_yaw = ceres::sin(yaw);
  //  Eigen::Matrix<T, 2, 2> rotation;
  //  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  //  return rotation;
  //}
  //
  //// return angle in [-pi,pi);
  // template <typename T> inline static T NormalizeAngle(const T &angle) {
  //  T two_pi(2.0 * M_PI);
  //  return angle - two_pi * ceres::floor((angle + T(M_PI)) / two_pi);
  //}
  //
  // template <typename T>
  // inline static T VectorCross2D(const Eigen::Matrix<T, 2, 1> &a,
  //                              const Eigen::Matrix<T, 2, 1> &b) {
  //  return a(0) * b(1) - a(1) * b(0);
  //}
  //
  // template <typename T>
  // inline static T VectorNorm2D(const Eigen::Matrix<T, 2, 1> &a) {
  //  return ceres::sqrt(a(0) * a(0) + a(1) * a(1));
  //}
  //
  // class AngleLocalParameterization {
  // public:
  //  template <typename T>
  //  bool operator()(const T *theta_radians, const T *delta_theta_radians,
  //                  T *theta_radians_plus_delta) const {
  //    *theta_radians_plus_delta =
  //        NormalizeAngle(*theta_radians + *delta_theta_radians);
  //
  //    return true;
  //  }
  //
  //  static ceres::LocalParameterization *Create() {
  //    return (
  //        new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
  //                                                 1, 1>);
  //  }
  //};

  class CeresPixelErrorTerm {
  public:
    CeresPixelErrorTerm(const Eigen::Vector3d &pt_src,
                        const Eigen::Vector2d &px_tgt,
                        const Eigen::Matrix2d &sqrt_info,
                        const CameraModel &cam)
        : point_src_(pt_src), pixel_tgt_(px_tgt), sqrt_information_(sqrt_info),
          camera_(cam) {}

    template <typename T>
    bool operator()(const T *const translation, const T *const rotation,
                    T *residual_ptr) const {

      Eigen::Map<const Eigen::Matrix<T, 3, 1>> trns(translation);
      Eigen::Map<const Eigen::Quaternion<T>> quat(rotation);

      // camera intrinsic parameters;
      const T f = T(camera_.getFocal());
      const T cx = T(camera_.getCenterx());
      const T cy = T(camera_.getCentery());

      // projected point in the target frame;
      const Eigen::Matrix<T, 3, 1> pt_tgt =
          quat * point_src_.template cast<T>() + trns;

      // projected pixel in the target image frame;
      const T v = f * pt_tgt[0] / pt_tgt[2] + cx;
      const T u = f * pt_tgt[1] / pt_tgt[2] + cy;
      const Eigen::Matrix<T, 2, 1> px_tgt(u, v);

      Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residual_ptr);
      residual = px_tgt - pixel_tgt_.template cast<T>();
      residual.applyOnTheLeft(sqrt_information_.template cast<T>());

      // Eigen::Quaternion<T> q_i_inv = q_i.conjugate();
      // Eigen::Quaternion<T> q_ij = q_i_inv * q_j;
      // Eigen::Matrix<T, 3, 1> p_ij = q_i_inv * (p_j - p_i);
      //
      // Eigen::Quaternion<T> q_ij_meas(pose_ij_meas_.linear().template
      // cast<T>());
      // Eigen::Quaternion<T> delta_q = q_ij_meas * q_ij.conjugate();
      //
      // Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residual_ptr);
      // Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_trs(residual_ptr);
      // Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_rot(residual_ptr + 3);
      //
      // residual_trs = p_ij - pose_ij_meas_.translation().template cast<T>();
      // residual_rot = T(2.0) * delta_q.vec();
      // residual.applyOnTheLeft(sqrt_information_.template cast<T>());
      //
      //  const Eigen::Matrix<T, 2, 1> translation(*x, *y);
      //  const Eigen::Matrix<T, 2, 2> rotation = RotationMatrix2D(*yaw);
      //
      //  Eigen::Matrix<T, 2, 1> trans_point =
      //      rotation * point.template cast<T>() + translation;
      //  Eigen::Matrix<T, 2, 1> p_ep_1 =
      //      trans_point - endpoint_1.template cast<T>();
      //  Eigen::Matrix<T, 2, 1> p_ep_2 =
      //      trans_point - endpoint_2.template cast<T>();
      //  Eigen::Matrix<T, 2, 1> ep1_ep2 =
      //      endpoint_1.template cast<T>() - endpoint_2.template cast<T>();
      //
      //  *residual_ptr = VectorCross2D(p_ep_1, p_ep_2) / VectorNorm2D(ep1_ep2);
      return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &pt_src,
                                       const Eigen::Vector2d &px_tgt,
                                       const Eigen::Matrix2d &sqrt_info,
                                       const CameraModel &cam) {
      return new ceres::AutoDiffCostFunction<CeresPixelErrorTerm, 2, 3, 4>(
          new CeresPixelErrorTerm(pt_src, px_tgt, sqrt_info, cam));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    const Eigen::Vector3d point_src_;
    const Eigen::Vector2d pixel_tgt_;
    const Eigen::Matrix2d sqrt_information_;
    const CameraModel camera_;
  };
};
} // namespace ZJL

#endif
