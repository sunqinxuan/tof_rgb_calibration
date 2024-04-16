/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-11-03 15:24
#
# Filename:		main_node.cpp
#
# Description:
#
************************************************/

#include <Eigen/Core>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "tic_toc.hpp"
#include <fstream>

using namespace std;
using namespace ZJL;

void downSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                const double ds_ratio, const int row_split,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pc_tof_trans(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pc_tof_ds(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tof(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_nav(new pcl::PointCloud<pcl::PointXYZ>);

bool flag = false;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *pc);
  flag = true;
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "calib", ros::init_options::AnonymousName);
  if (!ros::ok())
    return 0;

  ros::NodeHandle nh;

  // pc - stereo point cloud loaded from pcd file;
  // pc_ds - downsampled stereo point cloud;
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/sun/pc_disp.pcd", *pc);
  downSample(pc, 13, 600, pc_ds);

  // for display
  downSample(pc, 4, 0, pc_nav);

  // publish downsampled stereo point cloud;
  ros::Publisher pub =
      nh.advertise<sensor_msgs::PointCloud2>("pc_disp_downsample", 7);
  sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*pc_nav, *msg_cloud_ptr);
  msg_cloud_ptr->header.frame_id = "map";

  ros::Publisher pub_tof =
      nh.advertise<sensor_msgs::PointCloud2>("pc_transformed_tof", 7);
  ros::Publisher pub_tof_org =
      nh.advertise<sensor_msgs::PointCloud2>("pc_original_tof", 7);

  // pc is now the original tof cloud;
  ros::Subscriber sub = nh.subscribe("/up_tof", 7, &callback);

  // Eigen::Matrix4f trans;
  // trans << 0.999754, -0.004601, -0.021692, 0.409432, 0.004235, 0.999849,
  //    -0.016881, 0.027343, 0.021767, 0.016785, 0.999622, 0.033028, 0.000000,
  //    0.000000, 0.000000, 1.000000;
  // 0.999571, -0.008913, -0.027910,  0.437724,
  // 0.008333,  0.999748, -0.020837,  0.031935,
  // 0.028089,  0.020595,  0.999393,  0.110995,
  // 0.000000,  0.000000,  0.000000,  1.000000;

  Eigen::Matrix4f trans_org;
  trans_org << 0.999861, -0.000969, -0.016663, 0.372630, 0.001085, 0.999975,
      0.006961, -0.003863, 0.016656, -0.006978, 0.999837, 0.001669, 0.000000,
      0.000000, 0.000000, 1.000000;
  Eigen::Matrix4f transform;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  TicToc time;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (flag) {
      downSample(pc, 4, 120, pc_tof_ds);

      pcl::transformPointCloud(*pc_tof_ds, *pc_tof_trans, trans_org);

      time.tic();
      icp.setInputTarget(pc_ds);
      icp.setInputSource(pc_tof_trans);
      icp.align(*pc_tof_trans);
      transform = icp.getFinalTransformation();

      std::cout << std::endl
                << "icp: runtime= " << time.toc() * 1000.0 << std::endl
                << "transformation=" << std::endl
                << transform << std::endl;

      pcl::transformPointCloud(*pc, *pc_tof, transform);

      sensor_msgs::PointCloud2Ptr msg_cloudtof_ptr(
          new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*pc_tof, *msg_cloudtof_ptr);
      msg_cloudtof_ptr->header.frame_id = "map";
      pub_tof.publish(*msg_cloudtof_ptr);

      pub.publish(*msg_cloud_ptr);
    }
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}

void downSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                const double ds_ratio, const int row_split,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
  int rows = floor((cloud_in->height - row_split) / ds_ratio);
  int cols = floor(cloud_in->height / ds_ratio);
  cloud_out->resize(rows * cols);
  cloud_out->height = rows;
  cloud_out->width = cols;
  // pc_ds->resize(140*200);//(512 * 512);
  // pc_ds->height = 140;//512;
  // pc_ds->width = 200;//512;
  //	ofstream fp;
  //	fp.open("/home/sun/debug.txt",ios::out);
  int tmp = floor(row_split / ds_ratio);
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      cloud_out->at(i, j) = cloud_in->at(i * ds_ratio, (j + tmp) * ds_ratio);
    }
  }
}
