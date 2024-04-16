/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-11-15 08:50
#
# Filename:		calib502.cpp
#
# Description:
#
************************************************/
#include "calib502.hpp"
#include "tic_toc.hpp"
#include <cstdlib>
#include <ctime>

namespace ZJL {
using namespace std;
Calib502::Calib502(const std::string &work_space_path) {

  remove("/home/sun/calibration502/debug.txt");
  fp.open("/home/sun/calibration502/debug.txt", std::ios::app);

  // std::string work_space_path, cfg502_path;
  // nh_.param<std::string>("work_space_path", work_space_path, "");

  // cfg502_path = work_space_path + "/data/projectionCfg502.yaml";
  // YAML::Node cfg502 = YAML::LoadFile(cfg502_path);
  // std::vector<double> tmp;
  // tmp = cfg502["tof"]["left_to_tof"]["data"].as<std::vector<double>>();
  // for (int i = 0; i < 4; i++) {
  //  for (int j = 0; j < 4; j++) {
  //    T_nav_left_tof_.matrix()(i, j) = tmp[i * 4 + j];
  //  }
  //}
  // std::cout << "T_nav_left_tof_=" << std::endl
  //          << T_nav_left_tof_.matrix() << std::endl;

  std::string config_file_path = work_space_path + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(config_file_path);
  canny_th1_tof_ = config["canny_th1_tof"].as<double>();
  canny_th2_tof_ = config["canny_th2_tof"].as<double>();
  canny_th1_nav_ = config["canny_th1_nav"].as<double>();
  canny_th2_nav_ = config["canny_th2_nav"].as<double>();
  orb_th_nav_multispec_ = config["orb_th_nav_multispec"].as<double>();
  orb_th_nav_ = config["orb_th_nav"].as<double>();

  registration_ptr_ =
      std::make_shared<CannyRegistration>(config["CannyRegistration"]);

	srand(time(0));
}

Calib502::~Calib502() { fp.close(); }

bool Calib502::runCalibTofNav(
    const cv::Mat &img_nav, const cv::Mat &img_tof_intensity,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_tof,
    const CameraModel &cam_nav, Eigen::Isometry3d &T_nav_tof) {

  TicToc time;

  cv::Mat img_nav_edge, img_tof_edge;
  if (!extractCannyEdge(img_nav, canny_th1_nav_, canny_th2_nav_, 3,
                        img_nav_edge)) {
    std::cout << "[runCalibTofNav] nav image: extracting canny edge fails!"
              << std::endl;
    return false;
  }
  if (!extractCannyEdge(img_tof_intensity, canny_th1_tof_, canny_th2_tof_, 3,
                        img_tof_edge)) {
    std::cout << "[runCalibTofNav] tof intensity image: extracting canny "
                 "edge fails!"
              << std::endl;
    return false;
  }

  // project tof cloud to nav camera to obtain the color information;
  // cloud_vis_1.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // visualizePointCloud(img_nav, point_cloud_tof, T_nav_tof, cam_nav,
  // cloud_vis_1);
  visualizeEdges(img_nav_edge, img_tof_edge, point_cloud_tof, T_nav_tof,
                 cam_nav, img_vis_1);
  std::cout << "T_nav_tof=" << std::endl << T_nav_tof.matrix() << std::endl;

  pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud_nav(
      new pcl::PointCloud<pcl::PointXY>);
  generatePixelCloud(img_nav_edge, pixel_cloud_nav);
  pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud_tof(
      new pcl::PointCloud<pcl::PointXY>);
  generatePixelCloud(img_tof_edge, pixel_cloud_tof);

  registration_ptr_->setInputTarget(pixel_cloud_nav);

  time.tic();
  Eigen::Isometry3d T_nav_tof1 = T_nav_tof;
  registration_ptr_->alignScans(pixel_cloud_tof, point_cloud_tof, cam_nav,
                                T_nav_tof1);
  double duration = time.toc() * 1000.0;
  std::cout << "calibNavTof runtime: " << duration << std::endl;

  double err1 = registration_ptr_->getReprojError(
      pixel_cloud_tof, point_cloud_tof, cam_nav, T_nav_tof, 80 + rand() % 50);
  double err2 = registration_ptr_->getReprojError(
      pixel_cloud_tof, point_cloud_tof, cam_nav, T_nav_tof1, 7 + rand() % 7);

  std::cout << BLUE << "reproj error offline: " << err1 << std::endl;
  std::cout << BLUE << "reproj error online:  " << err2 << RESET << std::endl;
  fp << duration << " " << err1 << " " << err2 << std::endl;

  // img_vis_2
  visualizeEdges(img_nav_edge, img_tof_edge, point_cloud_tof, T_nav_tof1,
                 cam_nav, img_vis_2);
  registration_ptr_->drawLines(img_vis_2, cam_nav);
  std::cout << "T_nav_tof1=" << std::endl << T_nav_tof1.matrix() << std::endl;

  // cloud_vis_2.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // visualizePointCloud(img_nav, point_cloud_tof, T_nav_tof1, cam_nav,
  // cloud_vis_2);

  // T_nav_tof = T_nav_tof1;

  return true;
}

bool Calib502::runCalibNavMultispec(
    const cv::Mat &img_left, const cv::Mat &img_right,
    const cv::Mat &img_multispec, const CameraModel &cam_left,
    const CameraModel &cam_right, const CameraModel &cam_multispec,
    const Eigen::Isometry3d &T_left_right, Eigen::Isometry3d &T_nav_multispec) {

  // key point matching - multispec & nav left;
  std::vector<cv::KeyPoint> key_points_left, key_points_right;
  std::vector<cv::DMatch> matches;
  matchKeyPoints(img_left, img_right, key_points_left, key_points_right,
                 matches);

  // draw features;
  cv::Mat img_feature_left, img_feature_right;
  cv::drawKeypoints(img_left, key_points_left, img_feature_left,
                    cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(img_right, key_points_right, img_feature_right,
                    cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);

  // good_matches:
  // index key_points_left & key_points_right
  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].distance <= orb_th_nav_) {
      good_matches.push_back(matches[i]);
    }
  }

  cv::Mat img_matches;
  cv::drawMatches(img_left, key_points_left, img_right, key_points_right,
                  good_matches, img_matches);
  img_vis_2 = img_matches;

  // inputs for triangulatePoints
  cv::Mat proj_left =
      (cv::Mat_<double>(3, 4) << cam_left.getFocal(), 0, cam_left.getCenterx(),
       0, 0, cam_left.getFocal(), cam_left.getCentery(), 0, 0, 0, 1, 0);
  cv::Mat proj_right = (cv::Mat_<double>(3, 4) << cam_right.getFocal(), 0,
                        cam_right.getCenterx(), 0, 0, cam_right.getFocal(),
                        cam_right.getCentery(), 0, 0, 0, 1, 0);
  Eigen::Matrix4d trans_right_left = T_left_right.inverse().matrix();
  cv::Mat trans(4, 4, CV_64F);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      trans.at<double>(i, j) = trans_right_left(i, j);
    }
  }
  proj_right = proj_right * trans;

  // inputs for triangulatePoints
  std::vector<cv::Point2f> pixels_left, pixels_right;
  for (int i = 0; i < good_matches.size(); i++) {
    pixels_left.push_back(key_points_left[good_matches[i].queryIdx].pt);
    pixels_right.push_back(key_points_right[good_matches[i].trainIdx].pt);
  }

  cv::Mat obj_points_homo;
  cv::triangulatePoints(proj_left, proj_right, pixels_left, pixels_right,
                        obj_points_homo);

  std::vector<Eigen::Vector3d> obj_points;
  std::vector<int> indices_kp_left, indices_kp_right;
  std::vector<Eigen::Vector2d> reproj_pixels_left, reproj_pixels_right;
  cv::Mat img_reproj_left = img_left, img_reproj_right = img_right;
  for (int i = 0; i < obj_points_homo.cols; i++) {

    if (fabs(obj_points_homo.at<float>(3, i)) < 0.00001)
      continue;
    Eigen::Vector3d pt_left(obj_points_homo.at<float>(0, i),
                            obj_points_homo.at<float>(1, i),
                            obj_points_homo.at<float>(2, i));
    pt_left /= obj_points_homo.at<float>(3, i);
    obj_points.push_back(pt_left);
    indices_kp_left.push_back(good_matches[i].queryIdx);
    indices_kp_right.push_back(good_matches[i].trainIdx);
    Eigen::Vector3d pt_right = T_left_right.inverse() * pt_left;

    Eigen::Vector2d u_left, u_right;
    if (cam_left.proj2Image(pt_left, u_left) &&
        cam_right.proj2Image(pt_right, u_right)) {
      reproj_pixels_left.push_back(u_left);
      reproj_pixels_right.push_back(u_right);

      cv::Point2f pt_l = key_points_left[good_matches[i].queryIdx].pt;
      cv::Point2f pt_r = key_points_right[good_matches[i].trainIdx].pt;
      cv::Point2f pt_ll(u_left[1], u_left[0]);
      cv::Point2f pt_rr(u_right[1], u_right[0]);

      cv::circle(img_reproj_left, pt_l, 7,
                 cv::Scalar(0, 0, 255)); // measured-red
      cv::circle(img_reproj_left, pt_ll, 7,
                 cv::Scalar(0, 255, 0)); // reproj-green
      cv::line(img_reproj_left, pt_l, pt_ll,
               cv::Scalar(255, 255, 255)); // line-white

      cv::circle(img_reproj_right, pt_r, 7,
                 cv::Scalar(0, 0, 255)); // measured-red
      cv::circle(img_reproj_right, pt_rr, 7,
                 cv::Scalar(0, 255, 0)); // reproj-green
      cv::line(img_reproj_right, pt_r, pt_rr,
               cv::Scalar(255, 255, 255)); // line-white
    }
  }

  cloud_vis_1.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int i = 0; i < obj_points.size(); i++) {
    pcl::PointXYZRGBA pt;
    pt.x = obj_points[i][0];
    pt.y = obj_points[i][1];
    pt.z = obj_points[i][2];
    cloud_vis_1->push_back(pt);
  }

  // cv::hconcat(img_reproj_left, img_reproj_right, img_vis_1);

  // keypoints in left and right img corresponding to obj_points;
  std::vector<cv::KeyPoint> obj_kp_left, obj_kp_right;
  for (int i = 0; i < obj_points.size(); i++) {
    obj_kp_left.push_back(key_points_left[indices_kp_left[i]]);
    obj_kp_right.push_back(key_points_right[indices_kp_right[i]]);
  }

  // TODO !!!
  cv::Mat img_multispec_ds;
  cv::pyrDown(img_multispec, img_multispec_ds,
              cv::Size(img_multispec.cols / 2, img_multispec.rows / 2));
  // img_vis_1 = img_multispec;
  // img_vis_2 = img_multispec_ds;
  //    cv::circle(img_vis_1, cv::Point2f(612.0,612.0), 10,
  //               cv::Scalar(255, 255, 255));
  //    cv::circle(img_vis_2, cv::Point2f(306.0,306.0), 10,
  //               cv::Scalar(255, 255, 255));
  // std::cout << MAGENTA << img_multispec.rows << "\t" << img_multispec.cols
  //          << RESET << std::endl;
  // std::cout << MAGENTA << img_multispec_ds.rows << "\t" <<
  // img_multispec_ds.cols
  //          << RESET << std::endl;
  // return true;

  std::vector<cv::KeyPoint> key_points_multispec;
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(img_multispec_ds, key_points_multispec);

  // cv::Mat img_feature_multispec, img_feature_multispec_ds;
  // std::vector<cv::KeyPoint> key_points_multispec_org = key_points_multispec;
  // for (int i = 0; i < key_points_multispec.size(); i++) {
  //  key_points_multispec_org[i].pt *= 2.0;
  //}
  // cv::drawKeypoints(img_multispec, key_points_multispec_org,
  //                  img_feature_multispec, cv::Scalar(255, 255, 255),
  //                  cv::DrawMatchesFlags::DEFAULT);
  // cv::drawKeypoints(img_multispec_ds, key_points_multispec,
  //                  img_feature_multispec_ds, cv::Scalar(255, 255, 255),
  //                  cv::DrawMatchesFlags::DEFAULT);
  //// cv::hconcat(img_feature_multispec, img_feature_multispec_ds, img_vis_1);
  // img_vis_1 = img_feature_multispec;
  // img_vis_2 = img_feature_multispec_ds;
  // return true;

  std::vector<cv::DMatch> matches_left_multispec, matches_right_multispec;
  //	std::cout<<RED<<obj_kp_left.size()<<RESET<<std::endl;
  matchKeyPoints2(img_left, img_multispec_ds, obj_kp_left, key_points_multispec,
                  matches_left_multispec);
  matchKeyPoints2(img_right, img_multispec_ds, obj_kp_right,
                  key_points_multispec, matches_right_multispec);
  //	std::cout<<RED<<obj_kp_left.size()<<RESET<<std::endl;

  std::vector<cv::DMatch> good_matches_left, good_matches_right;
  for (int i = 0; i < matches_left_multispec.size(); i++) {
    if (matches_left_multispec[i].distance <= orb_th_nav_multispec_) {
      good_matches_left.push_back(matches_left_multispec[i]);
    }
  }
  for (int i = 0; i < matches_right_multispec.size(); i++) {
    if (matches_right_multispec[i].distance <= orb_th_nav_multispec_) {
      good_matches_right.push_back(matches_right_multispec[i]);
    }
  }

  for (int i = 0; i < key_points_multispec.size(); i++) {
    key_points_multispec[i].pt.x *= 2.0;
    key_points_multispec[i].pt.y *= 2.0;
  }

  cv::Mat img_matches_left, img_matches_right;
  cv::drawMatches(img_left, obj_kp_left, img_multispec, key_points_multispec,
                  good_matches_left, img_matches_left);
  // cv::drawMatches(img_right, obj_kp_right, img_multispec_ds,
  //                key_points_multispec, good_matches_right,
  //                img_matches_right);

  img_vis_1 = img_matches_left;
  // img_vis_2 = img_matches_right;

  /*
std::vector<cv::Point2f> pixels_kp_left;
std::vector<cv::Point2f> pixels_multispec;
for (int i = 0; i < good_matches_left.size(); i++) {
int idx_left = good_matches_left[i].queryIdx;
int idx_ms = good_matches_left[i].trainIdx;

          pixels_kp_left.push_back(obj_kp_left[i].pt);

cv::Point2d px(key_points_multispec[idx_ms].pt.x * 2.0,
             key_points_multispec[idx_ms].pt.y * 2.0);
pixels_multispec.push_back(px);
}
  if(pixels_kp_left.size()<=4) return false;

  cv::Mat homography=
  cv::findHomography(pixels_kp_left,pixels_multispec,cv::RANSAC);
  if(homography.empty()) return false;
  std::cout<<RED<<homography<<RESET<<std::endl;

std::vector<cv::Point2f> pixels_kp_left_ms;
  for(int i=0;i<pixels_kp_left.size();i++)
  {
          cv::Point2f pt=mulMatPoint(homography,pixels_kp_left[i]);
          cv::Point2f p=pixels_multispec[i];

          pixels_kp_left_ms.push_back(pt);
          std::cout<<CYAN<<p.x<<", "<<p.y<<RESET<<std::endl;
          std::cout<<CYAN<<pt.x<<", "<<pt.y<<RESET<<std::endl;
          std::cout<<CYAN<<p.x/pt.x<<", "<<p.y/pt.y<<RESET<<std::endl;

          cv::circle(img_multispec,pixels_multispec[i],7,cv::Scalar(255,255,255));
          cv::circle(img_multispec,pixels_kp_left_ms[i],7,cv::Scalar(0,0,0));
          cv::line(img_multispec,pixels_multispec[i],pixels_kp_left_ms[i],cv::Scalar(100,100,100));
  }
  img_vis_2=img_multispec;

cv::Mat cam_mat_ms = (cv::Mat_<double>(3, 3) << cam_multispec.getFocal(), 0,
                  cam_multispec.getCenterx(), 0, cam_multispec.getFocal(),
                  cam_multispec.getCentery(), 0, 0, 1);
cv::Mat cam_mat_left = (cv::Mat_<double>(3, 3) << cam_left.getFocal(), 0,
                  cam_left.getCenterx(), 0, cam_left.getFocal(),
                  cam_left.getCentery(), 0, 0, 1);
  cam_mat_left=cam_mat_left.inv();

  Eigen::Matrix3d R=T_nav_multispec.linear();
  Eigen::Vector3d t=T_nav_multispec.translation();
  cv::Mat rotation = (cv::Mat_<double>(3,3)
<<R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2)); cv::Mat
translation = (cv::Mat_<double>(3,1) <<t[0],t[1],t[2]);
  */

  // solvePnP
  // the points are approximately coplanar;
  // degenerate solutions;
  std::vector<cv::Point3d> obj_points_cv;
  std::vector<cv::Point2d> pixels_multispec;
  for (int i = 0; i < good_matches_left.size(); i++) {
    int idx_left = good_matches_left[i].queryIdx;
    int idx_ms = good_matches_left[i].trainIdx;

    cv::Point3d pt(obj_points[idx_left][0], obj_points[idx_left][1],
                   obj_points[idx_left][2]);
    obj_points_cv.push_back(pt);

    cv::Point2d px(key_points_multispec[idx_ms].pt.x * 2.0,
                   key_points_multispec[idx_ms].pt.y * 2.0);
    pixels_multispec.push_back(px);
  }
  if (obj_points_cv.size() <= 4)
    return false;
  // for (int i = 0; i < good_matches_right.size(); i++) {
  //  int idx_right = good_matches_right[i].queryIdx;
  //  int idx_ms = good_matches_right[i].trainIdx;
  //
  //  cv::Point3d pt(obj_points[idx_right][0], obj_points[idx_right][1],
  //                 obj_points[idx_right][2]);
  //  obj_points_cv.push_back(pt);
  //
  //  cv::Point2d px(key_points_multispec[idx_ms].pt.x * 2.0,
  //                 key_points_multispec[idx_ms].pt.y * 2.0);
  //  pixels_multispec.push_back(px);
  //}

  cv::Mat cam_mat_ms = (cv::Mat_<double>(3, 3) << cam_multispec.getFocal(), 0,
                        cam_multispec.getCenterx(), 0, cam_multispec.getFocal(),
                        cam_multispec.getCentery(), 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64FC1);

  Eigen::Isometry3d T_ms_nav = T_nav_multispec.inverse();
  Eigen::Vector3d axis_angle = Rotation2AxisAngle(T_ms_nav.linear());

  cv::Mat rvec =
      (cv::Mat_<double>(3, 1) << axis_angle[0], axis_angle[1], axis_angle[2]);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << T_ms_nav.translation()[0],
                  T_ms_nav.translation()[1], T_ms_nav.translation()[2]);

  std::cout << BLUE << rvec.t() << RESET << std::endl;
  std::cout << BLUE << tvec.t() << RESET << std::endl;
  cv::solvePnP(obj_points_cv, pixels_multispec, cam_mat_ms, dist_coeffs, rvec,
               tvec);
  std::cout << RED << rvec.t() << RESET << std::endl;
  std::cout << RED << tvec.t() << RESET << std::endl;

  return true;
}

/*
bool Calib502::runCalibNavMultispec(const cv::Mat &img_nav_left,
                                    const cv::Mat &img_nav_right,
                                    const cv::Mat &img_multispec,
                                    const CameraModel &cam_nav_left,
                                    const CameraModel &cam_nav_right,
                                    const CameraModel &cam_multispec,
                                    const Eigen::Isometry3d &T_nav_left_right,
                                    Eigen::Isometry3d &T_nav_multispec) {
  cv::Rect rect(700, 700, 700, 700);
  //	cv::Rect rect(550,550,1024,1024);
  cv::Mat img_nav_left_rect = img_nav_left(rect);
  rect = cv::Rect(600, 700, 700, 700);
  cv::Mat img_nav_right_rect = img_nav_right(rect);

  cv::Mat img_multispec_ds;
  cv::pyrDown(img_multispec, img_multispec_ds,
              cv::Size(img_multispec.cols / 2, img_multispec.rows / 2));

  // key point matching - multispec & nav left;
  std::vector<cv::KeyPoint> key_points_nav_left, key_points_multispec;
  std::vector<cv::DMatch> matches_nav_left_multispec;
  matchKeyPoints(img_nav_left_rect, img_multispec_ds, key_points_nav_left,
                 key_points_multispec, matches_nav_left_multispec);

  cv::Mat img_feature_nav_left, img_feature_multispec;
  cv::drawKeypoints(img_nav_left_rect, key_points_nav_left,
                    img_feature_nav_left, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(img_multispec_ds, key_points_multispec,
                    img_feature_multispec, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DEFAULT);

  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < matches_nav_left_multispec.size(); i++) {
    //		std::cout<<i<<"\t"<<matches[i].distance<<std::endl;
    if (matches_nav_left_multispec[i].distance <= orb_th_nav_multispec_) {
      good_matches.push_back(matches_nav_left_multispec[i]);
    }
  }

  cv::Mat img_matches;
  cv::drawMatches(img_nav_left_rect, key_points_nav_left, img_multispec_ds,
                  key_points_multispec, good_matches, img_matches);

  // key point matching - multispec & nav right;
  std::vector<cv::KeyPoint> key_points_nav_right, key_points_multispec1;
  std::vector<cv::DMatch> matches_nav_right_multispec;
  matchKeyPoints(img_nav_right_rect, img_multispec_ds, key_points_nav_right,
                 key_points_multispec1, matches_nav_right_multispec);

  cv::Mat img_feature_nav_right, img_feature_multispec1;
  cv::drawKeypoints(img_nav_right_rect, key_points_nav_right,
                    img_feature_nav_right, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(img_multispec_ds, key_points_multispec1,
                    img_feature_multispec1, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DEFAULT);

  std::vector<cv::DMatch> good_matches1;
  for (int i = 0; i < matches_nav_right_multispec.size(); i++) {
    //		std::cout<<i<<"\t"<<matches[i].distance<<std::endl;
    if (matches_nav_right_multispec[i].distance <= orb_th_nav_multispec_) {
      good_matches1.push_back(matches_nav_right_multispec[i]);
    }
  }

  cv::Mat img_matches1;
  cv::drawMatches(img_nav_right_rect, key_points_nav_right, img_multispec_ds,
                  key_points_multispec1, good_matches1, img_matches1);

  img_vis_1 = img_feature_nav_right;
  img_vis_2 = img_matches1;

  return true;
}
*/

bool Calib502::generatePixelCloud(
    const cv::Mat &img_edge, pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud) {
  if (img_edge.empty())
    return false;

  for (int i = 0; i < img_edge.rows; i++) {
    for (int j = 0; j < img_edge.cols; j++) {
      if (img_edge.at<unsigned char>(i, j) == 0)
        continue;
      pcl::PointXY pt;
      pt.x = (float)i;
      pt.y = (float)j;
      pixel_cloud->push_back(pt);
    }
  }

  return true;
}

bool Calib502::extractCannyEdge(const cv::Mat &img, const double canny_th1,
                                const double canny_th2, const double blur_size,
                                cv::Mat &img_edge) {
  if (img.empty())
    return false;

  cv::Mat img_blur;
  if (img.channels() == 1) {
    cv::blur(img, img_blur, cv::Size(blur_size, blur_size));
  } else if (img.channels() == 3) {
    cv::cvtColor(img, img_blur, CV_BGR2GRAY);
    cv::blur(img_blur, img_blur, cv::Size(blur_size, blur_size));
  } else {
    std::cout << "[Calib502][extractCannyEdge] input image is neither "
                 "1-channel nor 3-channel."
              << std::endl;
    return false;
  }
  cv::Canny(img_blur, img_edge, canny_th1, canny_th2, 3);

  return true;
}

bool Calib502::visualizeEdges(
    const cv::Mat &img_nav, const cv::Mat &img_tof,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_tof,
    const Eigen::Isometry3d &T_nav_tof, const CameraModel &cam,
    cv::Mat &img_edges) {
  // img_edges
  // draw edges extrated from nav_left to img_edges for display;
  img_edges.create(img_nav.rows, img_nav.cols, CV_8UC3);
  for (int i = 0; i < img_nav.rows; i++) {
    for (int j = 0; j < img_nav.cols; j++) {
      img_edges.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      if (img_nav.at<unsigned char>(i, j) == 0)
        continue;
      img_edges.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0); // green;
    }
  }

  // project the edges in tof to nav_left;
  for (int i = 0; i < img_tof.rows; i++) {
    for (int j = 0; j < img_tof.cols; j++) {
      if (img_tof.at<unsigned char>(i, j) == 0)
        continue;

      const pcl::PointXYZI &pt = cloud_tof->at(j, i);
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      p = T_nav_tof * p;
      Eigen::Vector2d px;
      if (cam.proj2Image(p, px)) {
        int u = floor(px[0]), v = floor(px[1]);
        img_edges.at<cv::Vec3b>(u, v) = cv::Vec3b(0, 0, 255);

        // dilation of the tof edges;
        for (int k = u - 1; k <= u + 1; k++) {
          for (int l = v - 1; l <= v + 1; l++) {
            if (k < 0 || k >= img_edges.rows || l < 0 || l >= img_edges.cols)
              continue;
            img_edges.at<cv::Vec3b>(k, l) = cv::Vec3b(0, 0, 255); // red
          }
        }
      }
    }
  }
  return true;
}

bool Calib502::visualizePointCloud(
    const cv::Mat &img_nav,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_tof,
    const Eigen::Isometry3d &T_nav_tof, const CameraModel &cam,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

  cloud->resize(cloud_tof->size());
  cloud->width = cloud_tof->width;
  cloud->height = cloud_tof->height;
  for (int i = 0; i < cloud_tof->height; i++) {
    for (int j = 0; j < cloud_tof->width; j++) {
      pcl::PointXYZRGBA &pt = cloud->at(j, i);
      pt.x = cloud_tof->at(j, i).x;
      pt.y = cloud_tof->at(j, i).y;
      pt.z = cloud_tof->at(j, i).z;
      if (pt.z < 1e-3) {
        pt.r = 0;
        pt.g = 0;
        pt.b = 0;
      } else {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        p = T_nav_tof * p;
        // pixel on image of nav_left;
        Eigen::Vector2d px;
        if (cam.proj2Image(p, px)) {
          int u = floor(px[0]), v = floor(px[1]);
          pt.b = img_nav.at<cv::Vec3b>(u, v)[0];
          pt.g = img_nav.at<cv::Vec3b>(u, v)[1];
          pt.r = img_nav.at<cv::Vec3b>(u, v)[2];
        } else {
          pt.r = 0;
          pt.g = 0;
          pt.b = 0;
        }
      }
    }
  }
  return true;
}

bool Calib502::matchKeyPoints(const cv::Mat &img_1, const cv::Mat &img_2,
                              std::vector<cv::KeyPoint> &key_points_1,
                              std::vector<cv::KeyPoint> &key_points_2,
                              std::vector<cv::DMatch> &matches) {
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(img_1, key_points_1);
  detector->detect(img_2, key_points_2);

  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Mat descriptors_1, descriptors_2;
  descriptor->compute(img_1, key_points_1, descriptors_1);
  descriptor->compute(img_2, key_points_2, descriptors_2);

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors_1, descriptors_2, matches);

  // void 	match (InputArray queryDescriptors, InputArray trainDescriptors,
  // std::vector< DMatch > &matches, InputArray mask=noArray()) const
  //	DMatch
  //	Public Attributes
  //	float 	distance
  //	int 	imgIdx  train image index More...
  //	int 	queryIdx query descriptor index More...
  //	int 	trainIdx train descriptor index More...

  return true;
}

bool Calib502::matchKeyPoints2(const cv::Mat &img_1, const cv::Mat &img_2,
                               std::vector<cv::KeyPoint> &key_points_1,
                               std::vector<cv::KeyPoint> &key_points_2,
                               std::vector<cv::DMatch> &matches) {
  // cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  // detector->detect(img_1, key_points_1);
  // detector->detect(img_2, key_points_2);

  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Mat descriptors_1, descriptors_2;
  descriptor->compute(img_1, key_points_1, descriptors_1);
  descriptor->compute(img_2, key_points_2, descriptors_2);

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors_1, descriptors_2, matches);

  // void 	match (InputArray queryDescriptors, InputArray trainDescriptors,
  // std::vector< DMatch > &matches, InputArray mask=noArray()) const
  //	DMatch
  //	Public Attributes
  //	float 	distance
  //	int 	imgIdx  train image index More...
  //	int 	queryIdx query descriptor index More...
  //	int 	trainIdx train descriptor index More...

  return true;
}

Eigen::Vector3d Calib502::Rotation2AxisAngle(const Eigen::Matrix3d &R) const {
  double cos_theta = 0.5 * (R.trace() - 1);
  if (cos_theta >= 1)
    cos_theta = 0.99999;
  if (cos_theta <= -1)
    cos_theta = -0.99999;
  double theta = acos(cos_theta);
  double sin_theta = sin(theta);

  if (fabs(sin_theta) < 0.0001)
    return Eigen::Vector3d::Zero();

  Eigen::Matrix3d r_skew = (R - R.transpose()) / (sin_theta * 2.0);
  // std::cout << BLUE << r_skew << RESET << std::endl;

  Eigen::Vector3d r(r_skew(2, 1), r_skew(0, 2), r_skew(1, 0));
  return r * theta;
}

cv::Point2f Calib502::mulMatPoint(const cv::Mat &mat,
                                  const cv::Point2f &pt) const {
  std::cout << YELLOW << std::endl;
  Eigen::Vector3f point(pt.x, pt.y, 1);
  std::cout << point.transpose() << std::endl;
  Eigen::Matrix3f matrix;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      matrix(i, j) = mat.at<double>(i, j);
    }
  }
  std::cout << matrix << std::endl;
  Eigen::Vector3f point1 = matrix * point;
  std::cout << point1.transpose() << std::endl;
  //	point1*=(point[0]/point1[0]);

  std::cout << point1[0] / point1[2] << ", " << point1[1] / point1[2]
            << std::endl;
  std::cout << RESET << std::endl;
  if (fabs(point1[2]) > 0.00001) {
    //		return cv::Point2f(point1[0],point1[1]);
    return cv::Point2f(point1[0] / point1[2], point1[1] / point1[2]);
  } else
    return cv::Point2f(0, 0);
}
} // namespace ZJL
