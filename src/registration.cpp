/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-10-28 09:19
#
# Filename:		registration.cpp
#
# Description:
#
************************************************/

#include "registration.hpp"
//#include "global_defination/message_print.hpp"
//#include "tools/convert_matrix.hpp"
//#include <pcl/common/transforms.h>

namespace ZJL {
CannyRegistration::CannyRegistration(const YAML::Node &node) {

  max_corr_dist_ = node["max_corr_dist"].as<double>();
  max_iterations_ = node["max_iterations"].as<int>();
  thres_rot_ = cos(node["thres_rot"].as<double>() * M_PI / 180.0);
  thres_trs_ = node["thres_trs"].as<double>();
  num_neighbors_cov_ = node["num_neighbors_cov"].as<int>();

  max_num_iterations_ = node["ceres"]["max_num_iterations"].as<unsigned int>();
  minimizer_progress_to_stdout_ =
      node["ceres"]["minimizer_progress_to_stdout"].as<bool>();
  std::string solver_type = node["ceres"]["solver_type"].as<std::string>();
  if (solver_type == "sparse_normal_cholesky") {
    linear_solver_type_ = ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (solver_type == "dense_normal_cholesky") {
    linear_solver_type_ = ceres::DENSE_NORMAL_CHOLESKY;
  } else if (solver_type == "sparse_schur") {
    linear_solver_type_ = ceres::SPARSE_SCHUR;
  } else if (solver_type == "dense_schur") {
    linear_solver_type_ = ceres::DENSE_SCHUR;
  } else {
    std::cout << "[registration] wrong linear solver type in config file!"
              << std::endl;
  }
  std::string trust_region_type =
      node["ceres"]["trust_region_type"].as<std::string>();
  if (trust_region_type == "LM") {
    trust_region_strategy_type_ = ceres::LEVENBERG_MARQUARDT;
  } else if (trust_region_type == "DogLeg") {
    trust_region_strategy_type_ = ceres::DOGLEG;
  } else {
    std::cout << "[registration] wrong trust region type in config file!"
              << std::endl;
  }
}

bool CannyRegistration::setInputTarget(const PixelCloudPtr &input_edge_target) {

  if (input_edge_target->size() < 10)
    return false;

  input_edge_tgt_ = input_edge_target;
  kdtree_.reset(new KdTree());
  kdtree_->setInputCloud(input_edge_tgt_);

  return true;
}

// bool CannyRegistration::CloudClassify(
//    const CloudData::CLOUD_PTR &input, VecCloud &group,
//    std::vector<bool> &empty, std::vector<VecLine> &group_lines,
//    std::vector<std::vector<int>> &group_line_indices) {
//  size_t point_num = input->points.size();
//  if (point_num < 10) {
//    ERROR("[SGICP][CloudClassify] not sufficient points in input cloud!");
//    return false;
//  }
//  group.resize(SEMANTIC_NUMS);
//  // group_kdtree.resize(SEMANTIC_NUMS);
//  empty.resize(SEMANTIC_NUMS);
//  // group_cov.resize(SEMANTIC_NUMS);
//  group_lines.resize(SEMANTIC_NUMS);
//  group_line_indices.resize(SEMANTIC_NUMS);
//
//  for (int i = 0; i < SEMANTIC_NUMS; i++) {
//    group[i].reset(new pcl::PointCloud<pcl::PointXY>());
//  }
//  for (size_t i = 0; i < point_num; i++) {
//    int class_id = floorf(input->points[i].intensity);
//    pcl::PointXY pt;
//    pt.x = input->points[i].x;
//    pt.y = input->points[i].y;
//    group[class_id]->points.push_back(pt);
//  }
//  for (int i = 0; i < SEMANTIC_NUMS; i++) {
//    group_lines[i].clear();
//    group_line_indices[i].clear();
//    if (group[i]->points.size() < 7) {
//      empty[i] = true;
//    } else {
//      // empty[i] = false;
//      // group_cov[i].reset(new VecMat);
//      // group_cov[i]->resize(group[i]->size());
//      // if (!ComputeCovariance(group[i], group_kdtree[i], group_cov[i]))
//      // empty[i] = true;
//      group_line_indices[i].resize(group[i]->size(), -1.0);
//      line_extract_ptr_->Extract(group[i], i, group_lines[i],
//                                 group_line_indices[i]);
//      // DEBUG(i, "\t", group_lines[i].size());
//
//      if (group_lines[i].size() < 7)
//        empty[i] = true;
//      else
//        empty[i] = false;
//
//    }
//  }
//  return true;
//}

bool CannyRegistration::computeCovariance(const PixelCloudPtr &cloud,
                                          const std::vector<int> &cloud_indices,
                                          const KdTreePtr &tree,
                                          VecMatPtr &covs) {
  if (static_cast<std::size_t>(num_neighbors_cov_) > cloud_indices.size()) {
    // std::cout << "[registration][computeCovariance] input cloud is too
    // small!"
    //          << std::endl;
    return false;
  }

  Eigen::Vector2d mean;
  std::vector<int> nn_indices;
  nn_indices.reserve(num_neighbors_cov_);
  std::vector<float> nn_dist_sq;
  nn_dist_sq.reserve(num_neighbors_cov_);

  if (covs->size() != cloud_indices.size()) {
    covs->resize(cloud_indices.size());
  }

  VecMat::iterator it_cov = covs->begin();
  for (int i = 0; i < cloud_indices.size(); i++, ++it_cov) {
    const pcl::PointXY &query = cloud->at(cloud_indices[i]);
    Eigen::Matrix2d &cov = *it_cov;
    cov.setZero();
    mean.setZero();

    // search for the num_neighbors_cov_ nearest neighbours;
    tree->nearestKSearch(query, num_neighbors_cov_, nn_indices, nn_dist_sq);

    for (int j = 0; j < num_neighbors_cov_; j++) {
      const pcl::PointXY &pt = (*cloud)[nn_indices[j]];
      mean(0) += pt.x;
      mean(1) += pt.y;
      // mean(2) += pt.z;
      // left-bottom triangle of cov matrix;
      cov(0, 0) += pt.x * pt.x;
      cov(1, 0) += pt.y * pt.x;
      cov(1, 1) += pt.y * pt.y;
      // cov(2, 0) += pt.z * pt.x;
      // cov(2, 1) += pt.z * pt.y;
      // cov(2, 2) += pt.z * pt.z;
    }
    mean /= static_cast<double>(num_neighbors_cov_);
    for (int k = 0; k < 2; k++) {
      for (int l = 0; l <= k; l++) {
        cov(k, l) /= static_cast<double>(num_neighbors_cov_);
        cov(k, l) -= mean[k] * mean[l];
        cov(l, k) = cov(k, l);
      }
    }
    // compute the SVD (symmetric -> EVD);
    // singular values sorted in decreasing order;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU);
    cov.setZero();
    Eigen::Matrix2d U = svd.matrixU();
    // reconstitute the covariance matrx with modified singular values;
    for (int k = 0; k < 2; k++) {
      Eigen::Vector2d col = U.col(k);
      // float v = sgicp_epsilon_;  // smallest two singular value replaced by
      // sgicp_epsilon_;
      // if (k == 0) v = 1.;        // biggest singular value replaced by 1;
      float v = 1.;
      if (k == 0)
        v = 0.001;
      cov += v * col * col.transpose();
    }
  }

  return true;
}

void CannyRegistration::drawLines(cv::Mat &image, const CameraModel &cam) {
  for (int i = 0; i < indices_src_.size(); i++) {
    int idx_src = indices_src_[i];
    int idx_tgt = indices_tgt_[i];
    int v = (int)input_edge_src_->at(idx_src).x;
    int u = (int)input_edge_src_->at(idx_src).y;
    pcl::PointXYZI &pt = input_point_cloud_->at(u, v);
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    p = transformation_ * p;
    Eigen::Vector2d px_src;
    camera_model_.proj2Image(p, px_src);
    cv::Point line_ep1(px_src[1], px_src[0]);
    cv::Point line_ep2(input_edge_tgt_->at(idx_tgt).y,
                       input_edge_tgt_->at(idx_tgt).x);
    cv::line(image, line_ep1, line_ep2, cv::Scalar(255, 255, 255));
  }
}

bool CannyRegistration::alignScans(const PixelCloudPtr &input_edge_source,
                                   const PointCloudPtr &input_point_cloud,
                                   const CameraModel &cam,
                                   Eigen::Isometry3d &T_tgt_src) {
  if (input_edge_source->size() < 10 || input_point_cloud->size() < 10)
    return false;

  input_edge_src_ = input_edge_source;
  input_point_cloud_ = input_point_cloud;
  camera_model_ = cam;

  int nr_iterations_ = 0;
  converged_ = false;
  transformation_ = T_tgt_src;

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  Eigen::Isometry3d transformation_prev;

  while (!converged_) {
    indices_src_.clear();
    indices_tgt_.clear();

    for (int i = 0; i < input_edge_src_->size(); i++) {
      // project the source edge to the target frame;
      int v = (int)input_edge_src_->at(i).x;
      int u = (int)input_edge_src_->at(i).y;
      pcl::PointXYZI &pt = input_point_cloud_->at(u, v);
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      p = transformation_ * p;
      Eigen::Vector2d px_src;
      if (!camera_model_.proj2Image(p, px_src))
        continue;

      // find the correspondences between the source and target;
      pcl::PointXY query;
      query.x = px_src[0];
      query.y = px_src[1];
      if (kdtree_->nearestKSearch(query, 1, nn_indices, nn_dists) == 0)
        continue;
      if (nn_dists[0] < max_corr_dist_) {
        indices_src_.push_back(i);
        indices_tgt_.push_back(nn_indices[0]);
      }
    }

    covariances_tgt_.reset(new VecMat);
    if (!computeCovariance(input_edge_tgt_, indices_tgt_, kdtree_,
                           covariances_tgt_))
      continue;

    // std::cout << std::endl
    //          << BOLDRED << "iteration " << nr_iterations_ << std::endl;
    // std::cout << YELLOW << "transformation_=" << std::endl
    //          << transformation_.matrix() << std::endl;

    transformation_prev = transformation_;

    ceres::Problem problem;
    Eigen::Vector3d trns = transformation_.translation();
    Eigen::Quaterniond quat(transformation_.linear());
    if (!BuildCeresProblem(&problem, indices_src_, indices_tgt_, trns, quat)) {
      std::cout << "[registration][alignScans] building ceres problem fails!"
                << std::endl;
      return false;
    }
    if (!SolveCeresProblem(&problem)) {
      std::cout << "[registration][alignScans] invalid ceres solution!"
                << std::endl;
      return false;
    }

    transformation_.linear() = quat.toRotationMatrix();
    transformation_.translation() = trns;

    // std::cout << BLUE << "transformation_=" << std::endl
    //          << transformation_.matrix() << std::endl;

    nr_iterations_++;

    if (isConverged(transformation_ * transformation_prev.inverse()) ||
        nr_iterations_ >= max_iterations_) {
      converged_ = true;
      T_tgt_src = transformation_;
    }
  }

  return true;
}

double CannyRegistration::getReprojError(const PixelCloudPtr &input_edge_source,
                                         const PointCloudPtr &input_point_cloud,
                                         const CameraModel &cam,
                                         Eigen::Isometry3d &T_tgt_src,
                                         double max_dist) {
  if (input_edge_source->size() < 10 || input_point_cloud->size() < 10)
    return -1;

  int nr_iterations_ = 0;
  converged_ = false;

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  double sum = 0.0;
  int cnt = 0;

  for (int i = 0; i < input_edge_source->size(); i++) {
    // project the source edge to the target frame;
    int v = (int)input_edge_source->at(i).x;
    int u = (int)input_edge_source->at(i).y;
    pcl::PointXYZI &pt = input_point_cloud->at(u, v);
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    p = T_tgt_src * p;
    Eigen::Vector2d px_src;
    if (!cam.proj2Image(p, px_src))
      continue;

    // find the correspondences between the source and target;
    pcl::PointXY query;
    query.x = px_src[0];
    query.y = px_src[1];
    if (kdtree_->nearestKSearch(query, 1, nn_indices, nn_dists) == 0)
      continue;
    if (nn_dists[0] < max_dist) {
      sum += sqrt(nn_dists[0]);
      cnt++;
    }
  }

  if (cnt == 0)
    return -1;

  return sum / cnt;
}

// pcl::PointXY CannyRegistration::TransformPointXY(const Eigen::Isometry2f
// &trans,
//                                                 const pcl::PointXY &point) {
//  Eigen::Vector2f tmp(point.x, point.y);
//  tmp = trans * tmp;
//  pcl::PointXY pt;
//  pt.x = tmp[0];
//  pt.y = tmp[1];
//  return pt;
//}
// pcl::PointXY CannyRegistration::TransformPointXY(const Eigen::Isometry2f
// &trans,
//                                                 const Eigen::Vector2f &point)
//                                                 {
//  Eigen::Vector2f tmp = trans * point;
//  pcl::PointXY pt;
//  pt.x = tmp[0];
//  pt.y = tmp[1];
//  return pt;
//}
// float CannyRegistration::Point2LineDistance(const CloudData::POINTXYZI &pt,
//                                            const LineFeature &line) {
//  Eigen::Vector3f p(pt.x, pt.y, pt.z);
//  // Eigen::Vector3f tmp1 = p - line.endpoint_1;
//  // Eigen::Vector3f tmp2 = p - line.endpoint_2;
//  // Eigen::Vector3f den = line.endpoint_1 - line.endpoint_2;
//  // return float(tmp1.cross(tmp2).norm() / den.norm());
//  return float(line.direction.cross(p - line.centroid).norm());
//}
//
bool CannyRegistration::isConverged(
    const Eigen::Isometry3d &delta_transformation) {
  // 1. The epsilon (difference) between the previous transformation and the
  // current estimated transformation
  // a. translation magnitude -- squaredNorm:
  float translation_sqr = delta_transformation.translation().squaredNorm();
  // b. rotation magnitude -- angle:
  // float cos_angle = (transformation.linear().trace() - 1.0f) / 2.0f;
  float cos_angle = delta_transformation.linear().trace() / 2.0f;
  if (cos_angle >= thres_rot_ && translation_sqr <= thres_trs_) {
    return true;
  }
  // 3. The relative sum of Euclidean squared errors is smaller than a user
  // defined threshold
  // Absolute
  // if (fabs(correspondences_cur_mse_ - correspondences_prev_mse_) <
  //    mse_threshold_absolute_) {
  //  return true;
  //}
  //// Relative
  // if (fabs(correspondences_cur_mse_ - correspondences_prev_mse_) /
  //        correspondences_prev_mse_ <
  //    mse_threshold_relative_) {
  //  return true;
  //}
  //
  // correspondences_prev_mse_ = correspondences_cur_mse_;
  return false;
}
// bool CannyRegistration::UpdateCorrMaxDist(const float &mean_dist,
//                                          const float &max_dist,
//                                          const size_t &pc_size,
//                                          const size_t &inline_size) {
//  if (pc_size < 10)
//    return false;
//  float inline_rate = static_cast<float>(float(inline_size) / pc_size);
//
//  if (inline_rate < 0.7)
//    return false;
//
//  float corr_dist =
//      mean_dist * (0.5 + inline_rate) + max_dist * (1 - inline_rate);
//  corr_dist = corr_dist > max_dist_ ? max_dist_ : corr_dist;
//  max_corr_dist_ = corr_dist < min_dist_ ? min_dist_ : corr_dist;
//
//  return true;
//}
bool CannyRegistration::BuildCeresProblem(ceres::Problem *problem,
                                          const std::vector<int> &indices_src,
                                          const std::vector<int> &indices_tgt,
                                          Eigen::Vector3d &trns,
                                          Eigen::Quaterniond &quat) {

  if (problem == NULL) {
    std::cout
        << "[registration][BuildCeresProblem] invalid optimization problem!"
        << std::endl;
    return false;
  }

  if (indices_src.size() < 10 || indices_tgt.size() < 10) {
    std::cout
        << "[registration][BuildCeresProblem] insufficient correspondences!"
        << std::endl;
    return false;
  }

  ceres::LossFunction *loss_function = NULL; // new ceres::HuberLoss(1.0);
  ceres::LocalParameterization *quat_param =
      new ceres::EigenQuaternionParameterization;

  for (int i = 0; i < indices_src.size(); i++) {

    int idx_src = indices_src[i];
    int idx_tgt = indices_tgt[i];

    pcl::PointXY &px_tgt = input_edge_tgt_->at(idx_tgt);
    int v = (int)input_edge_src_->at(idx_src).x;
    int u = (int)input_edge_src_->at(idx_src).y;
    pcl::PointXYZI &pt_src = input_point_cloud_->at(u, v);

    Eigen::Vector2d px_t(px_tgt.x, px_tgt.y);
    Eigen::Vector3d pt_s(pt_src.x, pt_src.y, pt_src.z);
    Eigen::Matrix2d sqrt_info =
        (*covariances_tgt_)[i]; // Eigen::Matrix2d::Identity();

    ceres::CostFunction *cost_function =
        CeresPixelErrorTerm::Create(pt_s, px_t, sqrt_info, camera_model_);
    problem->AddResidualBlock(cost_function, loss_function, trns.data(),
                              quat.coeffs().data());
    problem->SetParameterization(quat.coeffs().data(), quat_param);
  }

  return true;
}

bool CannyRegistration::SolveCeresProblem(ceres::Problem *problem) {
  if (problem == NULL) {
    std::cout
        << "[registration][SolveCeresProblem] invalid optimization problem!"
        << std::endl;
    return false;
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_num_iterations_;
  options.linear_solver_type = linear_solver_type_;
  options.trust_region_strategy_type = trust_region_strategy_type_;
  options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  if (options.minimizer_progress_to_stdout)
    std::cout << summary.FullReport() << std::endl;

  return summary.IsSolutionUsable();
}

// CannyRegistration::Matrix6f CannyRegistration::GetCovariance() {
//  float cos_theta = transformation_.linear()(0, 0);
//  float sin_theta = transformation_.linear()(1, 0);
//  Eigen::Matrix3f d2E_dxi2 = Eigen::Matrix3f::Zero();
//  int cnt = 0;
//  cost_min_ = 0.0;
//  Eigen::Matrix2f scatter = Eigen::Matrix2f::Zero();
//  for (std::size_t i = 0; i < source_indices_group_.size(); i++) {
//    cnt += source_indices_group_[i].size();
//    for (std::size_t j = 0; j < source_indices_group_[i].size(); j++) {
//      std::size_t idx_src = source_indices_group_[i][j];
//      std::size_t idx_tgt = target_indices_group_[i][j];
//
//      const LineFeature2D &line_src = input_source_group_lines_[i][idx_src];
//      Eigen::Vector2f pt = line_src.centroid;
//      Eigen::Vector2f pt1 = transformation_ * pt;
//      const LineFeature2D &line_tgt = input_target_group_lines_[i][idx_tgt];
//      Eigen::Vector2f ep1 = line_tgt.endpoint_1;
//      Eigen::Vector2f ep2 = line_tgt.endpoint_2;
//
//      Eigen::Vector2f p_ep_1 = pt1 - ep1;
//      Eigen::Vector2f p_ep_2 = pt1 - ep2;
//      Eigen::Vector2f ep1_ep2 = ep1 - ep2;
//      float le = ep1_ep2.norm();
//      float dE = VectorCross2D(p_ep_1, p_ep_2) / le;
//      cost_min_ += dE * dE;
//
//      Eigen::Matrix3f dL_dxi_2;
//      // float xe1 = ep1[0], ye1 = ep1[1], xe2 = ep2[0], ye2 = ep2[1];
//      float x = pt[0], y = pt[1];
//      float ye = ep1[1] - ep2[1];
//      float xe = ep2[0] - ep1[0];
//      float r1 = -x * sin_theta - y * cos_theta;
//      float r2 = x * cos_theta - y * sin_theta;
//
//      dL_dxi_2(0, 0) = ye * ye; //(ye1 - ye2) * (ye1 - ye2);
//      dL_dxi_2(1, 0) = xe * ye; //(xe2 - xe1) * (ye1 - ye2);
//      dL_dxi_2(2, 0) = dL_dxi_2(0, 0) * r1 + dL_dxi_2(1, 0) * r2;
//
//      dL_dxi_2(0, 1) = dL_dxi_2(1, 0);
//      dL_dxi_2(1, 1) = xe * xe; //(xe2 - xe1) * (xe2 - xe1);
//      dL_dxi_2(2, 1) = dL_dxi_2(0, 1) * r1 + dL_dxi_2(1, 1) * r2;
//
//      dL_dxi_2(0, 2) = dL_dxi_2(2, 0);
//      dL_dxi_2(1, 2) = dL_dxi_2(2, 1);
//      dL_dxi_2(2, 2) = (ye * r1 + xe * r2) * (ye * r1 + xe * r2);
//
//      // Eigen::Matrix3f d2L_dxi2 = Eigen::Matrix3f::Zero();
//      // d2L_dxi2(2, 2) = ye * (-x * cos_theta + y * sin_theta) + xe * (-x *
//      // sin_theta - y * cos_theta);
//
//      // d2E_dxi2 += (1.0 / le) * ((1.0 / le) * dL_dxi_2 + dE * d2E_dxi2);
//      d2E_dxi2 += dL_dxi_2 / (le * le);
//
//      scatter += ep1_ep2 * ep1_ep2.transpose();
//    }
//  }
//  d2E_dxi2 *= 2.0 / float(cnt);
//  cost_min_ /= float(cnt - 3);
//
//  Eigen::Matrix<float, 3, 6> selection = Eigen::Matrix<float, 3, 6>::Zero();
//  selection(0, 0) = 1.0;
//  selection(1, 1) = 1.0;
//  selection(2, 5) = 1.0;
//
//  hessian_ = selection.transpose() * d2E_dxi2 * selection;
//
//  Eigen::SelfAdjointEigenSolver<Matrix6f> es;
//  es.compute(hessian_);
//  Vector6f hessian_eigenvalues_ = es.eigenvalues().cwiseAbs();
//  Matrix6f hessian_eigenvectors_ = es.eigenvectors();
//  Matrix6f eigenvalues_inv = Matrix6f::Zero();
//  for (int i = 0; i < 6; i++) {
//    if (hessian_eigenvalues_(i) > 1e-7) {
//      eigenvalues_inv(i, i) = 1.0 / hessian_eigenvalues_(i);
//    }
//  }
//  has_hessian_computed_ = true;
//
//  Matrix6f hessian_inv = hessian_eigenvectors_ * eigenvalues_inv *
//                         hessian_eigenvectors_.transpose();
//  Matrix6f cov = hessian_inv * cost_min_;
//
//  // DEBUG("[PLICP2D]");
//  // std::cout << "[registration] cost_min_ = " << cost_min_ << std::endl;
//  // std::cout << "[registration] hessian_ = " << std::endl << hessian_ <<
//  // std::endl;
//  // std::cout << "[registration] hessian_inv = " << std::endl << hessian_inv
//  <<
//  // std::endl;
//  // std::cout << "[registration] cov = " << std::endl << cov << std::endl <<
//  // std::endl;
//
//  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> ess;
//  ess.compute(scatter);
//  Eigen::Vector2f scatter_eigenvalues_ = ess.eigenvalues().cwiseAbs();
//  Eigen::Matrix2f scatter_eigenvectors_ = ess.eigenvectors();
//
//  DEBUG("scatter");
//  for (int i = 0; i < 2; i++) {
//    std::cout << scatter_eigenvalues_[i] << "\t"
//              << scatter_eigenvectors_.col(i).transpose() << std::endl;
//  }
//  std::cout << std::endl;
//
//  return cov;
//}
} // namespace ZJL
