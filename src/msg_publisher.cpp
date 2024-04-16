/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2022-10-31 14:33
#
# Filename: msg_publisher.cpp
#
# Description:
#
************************************************/

#include "msg_publisher.hpp"

namespace ZJL {

MsgPublisher::MsgPublisher(ros::NodeHandle &nh) : nh_(nh) {
  publisher_img_1 = nh_.advertise<sensor_msgs::Image>("calib502_image_1", 7);
  publisher_img_2 = nh_.advertise<sensor_msgs::Image>("calib502_image_2", 7);
  publisher_cloud_1 =
      nh_.advertise<sensor_msgs::PointCloud2>("calib502_pointcloud_1", 7);
  publisher_cloud_2 =
      nh_.advertise<sensor_msgs::PointCloud2>("calib502_pointcloud_2", 7);
}

void MsgPublisher::publishImage1(const cv::Mat &img, const double time) {
  if (img.empty())
    return;
  std::string type;
  if (img.type() == CV_8UC3)
    type = sensor_msgs::image_encodings::BGR8;
  else if (img.type() == CV_8UC1)
    type = sensor_msgs::image_encodings::MONO8;
  else
    std::cout << "[publisher] undefined image type!" << std::endl;
  cv_bridge::CvImage cv_img(std_msgs::Header(), type, img);
  sensor_msgs::ImagePtr msg_img_ptr = cv_img.toImageMsg();
  msg_img_ptr->header.stamp = ros::Time().fromSec(time);
  publisher_img_1.publish(*msg_img_ptr);
}

void MsgPublisher::publishImage2(const cv::Mat &img, const double time) {
  if (img.empty())
    return;
  std::string type;
  if (img.type() == CV_8UC3)
    type = sensor_msgs::image_encodings::BGR8;
  else if (img.type() == CV_8UC1)
    type = sensor_msgs::image_encodings::MONO8;
  else
    std::cout << "[publisher] undefined image type!" << std::endl;
  cv_bridge::CvImage cv_img(std_msgs::Header(), type, img);
  // cv_bridge::CvImage cv_img(std_msgs::Header(),
  // sensor_msgs::image_encodings::BGR8, img);
  sensor_msgs::ImagePtr msg_img_ptr = cv_img.toImageMsg();
  msg_img_ptr->header.stamp = ros::Time().fromSec(time);
  publisher_img_2.publish(*msg_img_ptr);
}

void MsgPublisher::publishPointCloud1(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const double time) {
  if (!cloud)
    return;
  sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud, *msg_cloud_ptr);
  msg_cloud_ptr->header.frame_id = "map";
  msg_cloud_ptr->header.stamp = ros::Time().fromSec(time);
  publisher_cloud_1.publish(*msg_cloud_ptr);
}

void MsgPublisher::publishPointCloud2(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const double time) {
  if (!cloud)
    return;
  sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud, *msg_cloud_ptr);
  msg_cloud_ptr->header.frame_id = "map";
  msg_cloud_ptr->header.stamp = ros::Time().fromSec(time);
  publisher_cloud_2.publish(*msg_cloud_ptr);
}
} // namespace ZJL
