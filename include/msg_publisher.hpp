/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2022-10-31 14:33
#
# Filename: msg_publisher.hpp
#
# Description:
#
************************************************/

#ifndef _MSG_PUBLISHER_HPP_
#define _MSG_PUBLISHER_HPP_

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "camera_model.hpp"

namespace ZJL {
class MsgPublisher {
public:
  MsgPublisher(ros::NodeHandle &nh);
  MsgPublisher() = default;

  void publishImage1(const cv::Mat &img, const double time);
  void publishImage2(const cv::Mat &img, const double time);
  void publishPointCloud1(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                          const double time);
  void publishPointCloud2(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                          const double time);

  // void Publish(const Eigen::Isometry3f &pose, const Eigen::Vector3f &vel,
  //             double time, bool status = false);
  // void Publish(const Eigen::Isometry3f &pose, const Eigen::Vector3f &vel,
  //             const Eigen::VectorXf &pvr_cov, double time,
  //             bool status = false);
  // void Publish(const Eigen::Isometry3f &pose, double time);
  // void Publish(const Eigen::Isometry3f &pose);
  //
  // bool HasSubscribers();

private:
  // void PublishData(const Eigen::Isometry3f &pose, ros::Time time);
  // void PublishData(const Eigen::Isometry3f &pose, const Eigen::Vector3f &vel,
  //                 ros::Time time, bool status = false);
  // void PublishData(const Eigen::Isometry3f &pose, const Eigen::Vector3f &vel,
  //                 const Eigen::VectorXf &pvr_cov, ros::Time time,
  //                 bool status = false);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_img_1, publisher_img_2;
  ros::Publisher publisher_cloud_1, publisher_cloud_2;
};
} // namespace ZJL
#endif
