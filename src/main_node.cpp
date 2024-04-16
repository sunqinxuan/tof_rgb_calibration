/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-11-15 08:50
#
# Filename:		main_node.cpp
#
# Description:
#
************************************************/
#include "calib502.hpp"
#include "msg_publisher.hpp"
#include "msg_subscriber.hpp"
#include "tic_toc.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

using namespace ZJL;

using namespace std;

void LoadYAML(const std::string &camera_config_path,
              const std::string &cam_type_1, const std::string &cam_type_2,
              CameraModel &cam1, CameraModel &cam2,
              Eigen::Isometry3d &transform_1_2);

int main(int argc, char **argv) {

  ros::init(argc, argv, "calib", ros::init_options::AnonymousName);
  if (!ros::ok())
    return 0;

  ros::NodeHandle nh;

  CameraModel cam_nav_left, cam_nav_right, cam_multispec, cam_ir, cam_tof;
  Eigen::Isometry3d T_nav_left_tof,
      T_nav_right_tof; //, T_multispec_tof, T_ir_tof;
  Eigen::Isometry3d T_nav_left_right, T_nav_multispec;

  std::string work_space_path;
  nh.param<std::string>("work_space_path", work_space_path, "");
  LoadYAML(work_space_path + "/config/config_sensors.yaml", "nav_left", "tof",
           cam_nav_left, cam_tof, T_nav_left_tof);
  LoadYAML(work_space_path + "/config/config_sensors.yaml", "nav_right", "tof",
           cam_nav_right, cam_tof, T_nav_right_tof);
  // LoadYAML(work_space_path + "/config/config_sensors.yaml", "ir", "tof",
  // cam_ir,
  //         cam_tof, T_ir_tof);
  // LoadYAML(work_space_path + "/config/config_sensors.yaml", "multispec",
  // "tof",
  //         cam_multispec, cam_tof, T_multispec_tof);

  // LoadYAML(work_space_path + "/config/config_sensors.yaml", "nav_left",
  //         "nav_right", cam_nav_left, cam_nav_right, T_nav_left_right);
  // LoadYAML(work_space_path + "/config/config_sensors.yaml", "nav_left",
  //         "multispec", cam_nav_left, cam_multispec, T_nav_multispec);

  Calib502 calib(work_space_path);
  MsgSubscriber msg_subscriber(nh);
  MsgPublisher msg_publisher(nh);

  cv::Mat img_nav_left, img_nav_right;
  cv::Mat img_tof_intensity;
  cv::Mat img_multispec;
  cv::Mat img_multispec_2, img_multispec_3, img_multispec_4;
  cv::Mat img_ir;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_tof(
      new pcl::PointCloud<pcl::PointXYZI>);

  ros::Time time;
  TicToc timer;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    // if (false)
    // if (msg_subscriber.synchronized())
    if (msg_subscriber.getImageNavLeft(img_nav_left) &&
        msg_subscriber.getImageTofIntensity(img_tof_intensity) &&
        msg_subscriber.getPointCloudTof(point_cloud_tof)) {

      calib.runCalibTofNav(img_nav_left, img_tof_intensity, point_cloud_tof,
                           cam_nav_left, T_nav_left_tof);

      time = ros::Time::now();
      msg_publisher.publishImage1(calib.img_vis_1, time.toSec());
      msg_publisher.publishImage2(calib.img_vis_2, time.toSec());
      msg_publisher.publishPointCloud1(calib.cloud_vis_1, time.toSec());
      msg_publisher.publishPointCloud2(calib.cloud_vis_2, time.toSec());
    }

    if (false)
      if (msg_subscriber.getImageNavRight(img_nav_right) &&
          msg_subscriber.getImageTofIntensity(img_tof_intensity) &&
          msg_subscriber.getPointCloudTof(point_cloud_tof)) {

        calib.runCalibTofNav(img_nav_right, img_tof_intensity, point_cloud_tof,
                             cam_nav_right, T_nav_right_tof);

        time = ros::Time::now();
        msg_publisher.publishImage1(calib.img_vis_1, time.toSec());
        msg_publisher.publishImage2(calib.img_vis_2, time.toSec());
        msg_publisher.publishPointCloud1(calib.cloud_vis_1, time.toSec());
        msg_publisher.publishPointCloud2(calib.cloud_vis_2, time.toSec());
      }

    // if (false)
    //  if (msg_subscriber.getImageMultispec(img_multispec) &&
    //      msg_subscriber.getImageTofIntensity(img_tof_intensity) &&
    //      msg_subscriber.getPointCloudTof(point_cloud_tof)) {
    //
    //    calib.runCalibTofMultispec(img_multispec, img_tof_intensity,
    //                               point_cloud_tof, cam_multispec,
    //                               T_multispec_tof);
    //
    //    time = ros::Time::now();
    //    msg_publisher.publishImage1(calib.img_vis_1, time.toSec());
    //    msg_publisher.publishImage2(calib.img_vis_2, time.toSec());
    //    msg_publisher.publishPointCloud1(calib.cloud_vis_1, time.toSec());
    //    msg_publisher.publishPointCloud2(calib.cloud_vis_2, time.toSec());
    //  }
    //
    // if (false)
    //  if (msg_subscriber.getImageIR(img_ir) &&
    //      msg_subscriber.getImageTofIntensity(img_tof_intensity) &&
    //      msg_subscriber.getPointCloudTof(point_cloud_tof)) {
    //
    //    calib.runCalibTofIR(img_ir, img_tof_intensity, point_cloud_tof,
    //    cam_ir,
    //                        T_ir_tof);
    //
    //    time = ros::Time::now();
    //    msg_publisher.publishImage1(calib.img_vis_1, time.toSec());
    //    msg_publisher.publishImage2(calib.img_vis_2, time.toSec());
    //    msg_publisher.publishPointCloud1(calib.cloud_vis_1, time.toSec());
    //    msg_publisher.publishPointCloud2(calib.cloud_vis_2, time.toSec());
    //  }

    if (false)
      if (msg_subscriber.synchronized())
        if (msg_subscriber.getImageMultispec(img_multispec) &&
            msg_subscriber.getImageNavLeft(img_nav_left) &&
            msg_subscriber.getImageNavRight(img_nav_right) &&
            msg_subscriber.getImageTofIntensity(img_tof_intensity) &&
            msg_subscriber.getPointCloudTof(point_cloud_tof)) {

          timer.tic();

          cv::Mat img_edges_align_left, img_edges_align_right, img_edges_align;
          calib.runCalibTofNav(img_nav_left, img_tof_intensity, point_cloud_tof,
                               cam_nav_left, T_nav_left_tof);
          img_edges_align_left = calib.img_vis_2;
          calib.runCalibTofNav(img_nav_right, img_tof_intensity,
                               point_cloud_tof, cam_nav_right, T_nav_right_tof);
          img_edges_align_right = calib.img_vis_2;
          cv::hconcat(img_edges_align_left, img_edges_align_right,
                      img_edges_align);

          Eigen::Isometry3d T_nav_left_right =
              T_nav_left_tof * T_nav_right_tof.inverse();
          calib.runCalibNavMultispec(img_nav_left, img_nav_right, img_multispec,
                                     cam_nav_left, cam_nav_right, cam_multispec,
                                     T_nav_left_right, T_nav_multispec);

          // calib.runCalibNavMultispec(img_nav_left, img_nav_right,
          // img_multispec,
          //                           cam_nav_left, cam_nav_right,
          //                           cam_multispec, T_nav_left_right,
          //                           T_nav_multispec);

          std::cout << "T_ms_nav:" << std::endl
                    << T_nav_multispec.inverse().matrix() << std::endl;
          std::cout << YELLOW << "runtime: " << timer.toc() * 1000.0 << RESET
                    << std::endl;

          time = ros::Time::now();
          msg_publisher.publishImage1(calib.img_vis_1, time.toSec());
          // msg_publisher.publishImage2(img_edges_align, time.toSec());
          msg_publisher.publishImage2(calib.img_vis_2, time.toSec());
          msg_publisher.publishPointCloud1(calib.cloud_vis_1, time.toSec());
          msg_publisher.publishPointCloud2(calib.cloud_vis_2, time.toSec());
        }

    if (false)
      if (msg_subscriber.getImageNavLeft(img_nav_left) &&
          msg_subscriber.getImageMultispec(img_multispec) &&
          msg_subscriber.getImageMultispec2(img_multispec_2) &&
          msg_subscriber.getImageMultispec3(img_multispec_3) &&
          msg_subscriber.getImageMultispec4(img_multispec_4)) {

        // cv::Mat channels[3] = {img_multispec, img_multispec_2,
        // img_multispec_3};
        // cv::Mat img_rgb;
        // cv::merge(channels, 3, img_rgb);

        cv::Mat img_gray;
        cv::cvtColor(img_nav_left, img_gray, CV_BGR2GRAY);

        // TODO
        // img_nav_left

        time = ros::Time::now();
        msg_publisher.publishImage1(img_nav_left, time.toSec());
        msg_publisher.publishImage2(img_gray, time.toSec());
      }

    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}

// LoadYAML
// cam_type_2 - tof camera
void LoadYAML(const std::string &camera_config_path,
              const std::string &cam_type_1, const std::string &cam_type_2,
              CameraModel &cam1, CameraModel &cam2,
              Eigen::Isometry3d &transform_1_2) {

  YAML::Node camera_config = YAML::LoadFile(camera_config_path);

  std::string name = "camera_" + cam_type_1;
  double f = camera_config[name]["focal"].as<double>();
  double cx = camera_config[name]["cx"].as<double>();
  double cy = camera_config[name]["cy"].as<double>();
  double px = camera_config[name]["pixelsize"].as<double>() * 1e-3;
  int width = camera_config[name]["width"].as<int>();
  int height = camera_config[name]["height"].as<int>();
  cam1 = CameraModel(f / px, f / px, cx / px + width / 2.0,
                     cy / px + height / 2.0, width, height);

  name = "camera_" + cam_type_2;
  f = camera_config[name]["focal"].as<double>();
  cx = camera_config[name]["cx"].as<double>();
  cy = camera_config[name]["cy"].as<double>();
  px = camera_config[name]["pixelsize"].as<double>() * 1e-3;
  width = camera_config[name]["width"].as<int>();
  height = camera_config[name]["height"].as<int>();
  cam2 = CameraModel(f / px, f / px, cx / px + width / 2.0,
                     cy / px + height / 2.0, width, height);

  Eigen::Isometry3d T_base_1;
  Eigen::Isometry3d T_base_2;

  name = "transform_base_" + cam_type_1;
  std::vector<double> tmp =
      camera_config[name]["data"].as<std::vector<double>>();
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(Calib502::deg2rad(tmp[3]), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(Calib502::deg2rad(tmp[4]), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(Calib502::deg2rad(tmp[5]), Eigen::Vector3d::UnitZ());
  T_base_1.linear() = q.toRotationMatrix();
  T_base_1.translation() = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) * 0.001; // m

  name = "transform_base_" + cam_type_2;
  tmp = camera_config[name]["data"].as<std::vector<double>>();
  q = Eigen::AngleAxisd(Calib502::deg2rad(tmp[3]), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(Calib502::deg2rad(tmp[4]), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(Calib502::deg2rad(tmp[5]), Eigen::Vector3d::UnitZ());
  T_base_2.linear() = q.toRotationMatrix();
  T_base_2.translation() = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) * 0.001;

  transform_1_2 = T_base_1.inverse() * T_base_2;
}
