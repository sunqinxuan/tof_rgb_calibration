/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-10-31 09:54
#
# Filename:		msg_subscriber.hpp
#
# Description:
#
************************************************/

#ifndef _MSG_SUBSCRIBER_HPP_
#define _MSG_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

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
class MsgSubscriber {
public:
  MsgSubscriber(ros::NodeHandle &nh);
  MsgSubscriber() = default;

  bool synchronized();

  bool getImageNavLeft(cv::Mat &img);
  bool getImageNavRight(cv::Mat &img);
  bool getImageTofIntensity(cv::Mat &img);
  bool getPointCloudTof(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  bool getImageIR(cv::Mat &img);
  bool getImageMultispec(cv::Mat &img);
  bool getImageMultispec2(cv::Mat &img);
  bool getImageMultispec3(cv::Mat &img);
  bool getImageMultispec4(cv::Mat &img);
  bool getImageMultispec5(cv::Mat &img);

private:
  void callbackNavigationLeft(const sensor_msgs::Image::ConstPtr &msg);
  void callbackNavigationRight(const sensor_msgs::Image::ConstPtr &msg);
  // void callbackTofRange(const sensor_msgs::Image::ConstPtr &msg);
  void callbackTofIntensity(const sensor_msgs::Image::ConstPtr &msg);
  void callbackTofCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void callbackIR(const sensor_msgs::Image::ConstPtr &msg);
  void callbackMultispec(const sensor_msgs::Image::ConstPtr &msg);
  void callbackMultispec2(const sensor_msgs::Image::ConstPtr &msg);
  void callbackMultispec3(const sensor_msgs::Image::ConstPtr &msg);
  void callbackMultispec4(const sensor_msgs::Image::ConstPtr &msg);
  void callbackMultispec5(const sensor_msgs::Image::ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  std::mutex buff_mutex_;

  ros::Subscriber sub_nav_left;
  ros::Subscriber sub_nav_right;
  ros::Subscriber sub_tof_intensity;
  ros::Subscriber sub_tof_cloud;
  ros::Subscriber sub_ir;
  ros::Subscriber sub_multispec;
  ros::Subscriber sub_multispec_2;
  ros::Subscriber sub_multispec_3;
  ros::Subscriber sub_multispec_4;
  ros::Subscriber sub_multispec_5;

  std::deque<double> deque_timestamp_nav_left_;
  std::deque<double> deque_timestamp_nav_right_;
  std::deque<double> deque_timestamp_tof_intensity_;
  std::deque<double> deque_timestamp_tof_cloud_;
  std::deque<double> deque_timestamp_ir_;
  std::deque<double> deque_timestamp_multispec_;
  std::deque<double> deque_timestamp_multispec_2_;
  std::deque<double> deque_timestamp_multispec_3_;
  std::deque<double> deque_timestamp_multispec_4_;
  std::deque<double> deque_timestamp_multispec_5_;

  std::deque<cv::Mat> deque_image_nav_left_;
  std::deque<cv::Mat> deque_image_nav_right_;
  std::deque<cv::Mat> deque_image_tof_intensity_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_point_cloud_tof_;
  std::deque<cv::Mat> deque_image_ir_;
  std::deque<cv::Mat> deque_image_multispec_;
  std::deque<cv::Mat> deque_image_multispec_2_;
  std::deque<cv::Mat> deque_image_multispec_3_;
  std::deque<cv::Mat> deque_image_multispec_4_;
  std::deque<cv::Mat> deque_image_multispec_5_;
};
} // namespace ZJL
#endif
