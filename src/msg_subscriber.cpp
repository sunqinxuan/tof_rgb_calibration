/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2022-10-31 09:54
#
# Filename:		msg_subscriber.cpp
#
# Description:
#
************************************************/
#include <msg_subscriber.hpp>

namespace ZJL {

MsgSubscriber::MsgSubscriber(ros::NodeHandle &nh) : nh_(nh) {
  std::string topic = "/Preposition_NavigationCamera_left";
  sub_nav_left =
      nh_.subscribe(topic, 7, &MsgSubscriber::callbackNavigationLeft, this);

  topic = "/Preposition_NavigationCamera_right";
  sub_nav_right =
      nh_.subscribe(topic, 7, &MsgSubscriber::callbackNavigationRight, this);

  // topic = "/up_tof_range";
  // ros::Subscriber sub_tof_range =
  //    nh_.subscribe(topic, 7, &MsgSubscriber::callbackTofRange, this);

  topic = "/up_tof_intensity";
  sub_tof_intensity =
      nh_.subscribe(topic, 7, &MsgSubscriber::callbackTofIntensity, this);

  topic = "/up_tof";
  sub_tof_cloud =
      nh_.subscribe(topic, 7, &MsgSubscriber::callbackTofCloud, this);

  // topic = "/ir_image";
  // sub_ir = nh_.subscribe(topic, 7, &MsgSubscriber::callbackIR, this);

  topic = "/multi_spectral0";
  sub_multispec =
      nh_.subscribe(topic, 7, &MsgSubscriber::callbackMultispec, this);

  // topic = "/multi_spectral2";
  // sub_multispec_2 =
  //    nh_.subscribe(topic, 7, &MsgSubscriber::callbackMultispec2, this);
  //
  // topic = "/multi_spectral3";
  // sub_multispec_3 =
  //    nh_.subscribe(topic, 7, &MsgSubscriber::callbackMultispec3, this);
  //
  // topic = "/multi_spectral4";
  // sub_multispec_4 =
  //    nh_.subscribe(topic, 7, &MsgSubscriber::callbackMultispec4, this);
  //
  // topic = "/multi_spectral5";
  // sub_multispec_5 =
  //    nh_.subscribe(topic, 7, &MsgSubscriber::callbackMultispec5, this);
}

bool MsgSubscriber::synchronized() {
  if (deque_image_nav_left_.size() == 0 || deque_image_nav_right_.size() == 0 ||
      deque_image_tof_intensity_.size() == 0 ||
      deque_point_cloud_tof_.size() == 0 || deque_image_multispec_.size() == 0)
    return false;

  double time_nav_left = deque_timestamp_nav_left_.front();
  double time_nav_right = deque_timestamp_nav_right_.front();
  double time_tof_intensity = deque_timestamp_tof_intensity_.front();
  double time_tof_cloud = deque_timestamp_tof_cloud_.front();
  double time_multispec = deque_timestamp_multispec_.front();

  double delta = 0.3;
  if (time_nav_right - time_nav_left > delta ||
      time_tof_intensity - time_nav_left > delta ||
      time_tof_cloud - time_nav_left > delta ||
      time_multispec - time_nav_left > delta) {
    std::cout << YELLOW << "dropped nav left: \t" << std::fixed << time_nav_left
              << RESET << std::endl;
    deque_timestamp_nav_left_.pop_front();
    deque_image_nav_left_.pop_front();
    return false;
  }

  std::cout << std::fixed << RED << "current nav left: \t" << time_nav_left
            << RESET << std::endl;

  while (time_nav_right < time_nav_left + delta) {
    if (fabs(time_nav_right - time_nav_left) < 0.1) {
      std::cout << std::fixed << GREEN << "sync nav right: \t" << time_nav_right
                << RESET << std::endl;
      break;
    }
    std::cout << std::fixed << YELLOW << "dropped nav right: \t"
              << time_nav_right << RESET << std::endl;
    deque_timestamp_nav_right_.pop_front();
    deque_image_nav_right_.pop_front();
    if (deque_timestamp_nav_right_.size() == 0)
      return false;
    time_nav_right = deque_timestamp_nav_right_.front();
  }

  while (time_tof_intensity < time_nav_left + delta) {
    if (fabs(time_tof_intensity - time_nav_left) < 0.1) {
      std::cout << std::fixed << GREEN << "sync tof intensity: \t"
                << time_tof_intensity << RESET << std::endl;
      break;
    }
    std::cout << std::fixed << YELLOW << "dropped tof intensity: \t"
              << time_tof_intensity << RESET << std::endl;
    deque_timestamp_tof_intensity_.pop_front();
    deque_image_tof_intensity_.pop_front();
    if (deque_timestamp_tof_intensity_.size() == 0)
      return false;
    time_tof_intensity = deque_timestamp_tof_intensity_.front();
  }

  while (time_tof_cloud < time_nav_left + delta) {
    if (fabs(time_tof_cloud - time_nav_left) < 0.1) {
      std::cout << std::fixed << GREEN << "sync tof cloud: \t" << time_tof_cloud
                << RESET << std::endl;
      break;
    }
    std::cout << std::fixed << YELLOW << "dropped tof cloud: \t"
              << time_tof_cloud << RESET << std::endl;
    deque_timestamp_tof_cloud_.pop_front();
    deque_point_cloud_tof_.pop_front();
    if (deque_timestamp_tof_cloud_.size() == 0)
      return false;
    time_tof_cloud = deque_timestamp_tof_cloud_.front();
  }

  while (time_multispec < time_nav_left + delta) {
    if (fabs(time_multispec - time_nav_left) < 0.1) {
      std::cout << std::fixed << GREEN << "sync multispec: \t" << time_multispec
                << RESET << std::endl;
      break;
    }
    std::cout << std::fixed << YELLOW << "dropped multispec: \t"
              << time_multispec << RESET << std::endl;
    deque_timestamp_multispec_.pop_front();
    deque_image_multispec_.pop_front();
    if (deque_timestamp_multispec_.size() == 0)
      return false;
    time_multispec = deque_timestamp_multispec_.front();
  }

  // struct {
  //  using value_type = std::pair<double, std::deque<double> *>;
  //  bool operator()(const value_type &a, const value_type &b) const {
  //    return a.first < b.first;
  //  }
  //} Comparator;

  // std::vector<std::pair<double, std::deque<double> *>> comp_time;
  // comp_time.push_back(std::make_pair(deque_timestamp_nav_left_.front(),
  //                                   &deque_timestamp_nav_left_));
  // comp_time.push_back(std::make_pair(deque_timestamp_nav_right_.front(),
  //                                   &deque_timestamp_nav_right_));
  // comp_time.push_back(std::make_pair(deque_timestamp_tof_intensity_.front(),
  //                                   &deque_timestamp_tof_intensity_));
  // comp_time.push_back(std::make_pair(deque_timestamp_tof_cloud_.front(),
  //                                   &deque_timestamp_tof_cloud_));
  // comp_time.push_back(std::make_pair(deque_timestamp_multispec_.front(),
  //                                   &deque_timestamp_multispec_));

  // for (int i = 0; i < comp_time.size(); i++) {
  //  std::cout << comp_time[i].first << "\t" << comp_time[i].second->front()
  //            << std::endl;
  //}

  return true;
}

bool MsgSubscriber::getImageNavLeft(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_nav_left_.size() == 0)
    flag = false;
  else {
    img = deque_image_nav_left_.front();
    deque_image_nav_left_.pop_front();
    flag = true;
    std::cout << "time image nav left: " << std::fixed
              << deque_timestamp_nav_left_.front() << std::endl;
    deque_timestamp_nav_left_.pop_front();
  }
  //
  // TODO
  // how long should I kept the queue?
  //
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageNavRight(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_nav_right_.size() == 0)
    flag = false;
  else {
    img = deque_image_nav_right_.front();
    deque_image_nav_right_.pop_front();
    flag = true;
    std::cout << "time image nav right: " << std::fixed
              << deque_timestamp_nav_right_.front() << std::endl;
    deque_timestamp_nav_right_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageTofIntensity(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = true;
  if (deque_image_tof_intensity_.size() == 0)
    flag = false;
  else {
    img = deque_image_tof_intensity_.front();
    deque_image_tof_intensity_.pop_front();
    flag = true;
    std::cout << "time image tof intensity: " << std::fixed
              << deque_timestamp_tof_intensity_.front() << std::endl;
    deque_timestamp_tof_intensity_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getPointCloudTof(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_point_cloud_tof_.size() == 0)
    flag = false;
  else {
    cloud = deque_point_cloud_tof_.front();
    deque_point_cloud_tof_.pop_front();
    flag = true;
    std::cout << "time image tof cloud: " << std::fixed
              << deque_timestamp_tof_cloud_.front() << std::endl;
    deque_timestamp_tof_cloud_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageIR(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_ir_.size() == 0)
    flag = false;
  else {
    img = deque_image_ir_.front();
    deque_image_ir_.pop_front();
    flag = true;
    std::cout << "time image ir: " << std::fixed << deque_timestamp_ir_.front()
              << std::endl;
    deque_timestamp_ir_.pop_front();
  }
  //
  // TODO
  // how long should I kept the queue?
  //
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageMultispec(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_multispec_.size() == 0)
    flag = false;
  else {
    img = deque_image_multispec_.front();
    deque_image_multispec_.pop_front();
    flag = true;
    std::cout << "time image multispectral: " << std::fixed
              << deque_timestamp_multispec_.front() << std::endl;
    deque_timestamp_multispec_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageMultispec2(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_multispec_2_.size() == 0)
    flag = false;
  else {
    img = deque_image_multispec_2_.front();
    deque_image_multispec_2_.pop_front();
    flag = true;
    std::cout << "time image multispectral 2: " << std::fixed
              << deque_timestamp_multispec_2_.front() << std::endl;
    deque_timestamp_multispec_2_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageMultispec3(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_multispec_3_.size() == 0)
    flag = false;
  else {
    img = deque_image_multispec_3_.front();
    deque_image_multispec_3_.pop_front();
    flag = true;
    std::cout << "time image multispectral 3: " << std::fixed
              << deque_timestamp_multispec_3_.front() << std::endl;
    deque_timestamp_multispec_3_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageMultispec4(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_multispec_4_.size() == 0)
    flag = false;
  else {
    img = deque_image_multispec_4_.front();
    deque_image_multispec_4_.pop_front();
    flag = true;
    std::cout << "time image multispectral 4: " << std::fixed
              << deque_timestamp_multispec_4_.front() << std::endl;
    deque_timestamp_multispec_4_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

bool MsgSubscriber::getImageMultispec5(cv::Mat &img) {
  buff_mutex_.lock();
  bool flag = false;
  if (deque_image_multispec_5_.size() == 0)
    flag = false;
  else {
    img = deque_image_multispec_5_.front();
    deque_image_multispec_5_.pop_front();
    flag = true;
    std::cout << "time image multispectral 5: " << std::fixed
              << deque_timestamp_multispec_5_.front() << std::endl;
    deque_timestamp_multispec_5_.pop_front();
  }
  buff_mutex_.unlock();
  return flag;
}

void MsgSubscriber::callbackNavigationLeft(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (!ptr_img->image.empty()) {
    deque_image_nav_left_.push_back(ptr_img->image);
    deque_timestamp_nav_left_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();

  // cv::Mat img_blur;
  // cv::cvtColor(image_nav_left_, img_blur, CV_BGR2GRAY);
  // cv::blur(img_blur, img_blur, cv::Size(3, 3));
  // cv::Canny(img_blur, image_nav_left_edge_, canny_th1_nav_, canny_th2_nav_,
  // 3);
  //
  // flag_nav_left_received_ = true;
  // timestamp_nav_left_ = msg->header.stamp.toSec();

  // cv::imwrite("/home/sun/nav_left.jpg", image_nav_left_);
  // cv::imwrite("/home/sun/nav_left_edge.jpg", image_nav_left_edge_);

  //		cv::namedWindow("nav_left_edge", cv::WINDOW_NORMAL);
  //		cv::imshow("nav_left_edge",image_nav_left_edge_);

  //		cv::Ptr<cv::FeatureDetector> detector=cv::ORB::create();
  //		std::vector<cv::KeyPoint> key_points;
  //		detector->detect(image_nav_left_,key_points);
  //
  //		cv::Mat img_feature;
  //		cv::drawKeypoints(image_nav_left_,key_points,img_feature,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DEFAULT);
  //
  //		cv::namedWindow("nav_left_feature", cv::WINDOW_NORMAL);
  //		cv::imshow("nav_left_feature",img_feature);

  //		cv::waitKey(1);
}

void MsgSubscriber::callbackNavigationRight(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (!ptr_img->image.empty()) {
    deque_image_nav_right_.push_back(ptr_img->image);
    deque_timestamp_nav_right_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
  // flag_nav_right_received_ = true;

  // cv::Mat img_blur;
  // cv::cvtColor(image_nav_left_, img_blur, CV_BGR2GRAY);
  // cv::blur(img_blur, img_blur, cv::Size(3, 3));
  // cv::Canny(img_blur, image_nav_right_edge_, canny_th1_nav_, canny_th2_nav_,
  // 3);
}

// void MsgSubscriber::callbackTofRange(const sensor_msgs::Image::ConstPtr &msg)
// {
//  cv_bridge::CvImageConstPtr ptr_img =
//      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
//  cv::Mat img = ptr_img->image;
//}

void MsgSubscriber::callbackTofIntensity(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_tof_intensity_.push_back(ptr_img->image);
    deque_timestamp_tof_intensity_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();

  // cv::Mat img_blur;
  // cv::blur(img, img_blur, cv::Size(3, 3));
  // cv::Canny(img_blur, image_tof_edge_, canny_th1_tof_, canny_th2_tof_, 3);
  //
  // flag_tof_intensity_received_ = true;
  // timestamp_tof_intensity_ = msg->header.stamp.toSec();

  //		cv::namedWindow("tof_edge", cv::WINDOW_NORMAL);
  //		cv::imshow("tof_edge",image_tof_edge_);
  // cv::imwrite("/home/sun/tof.jpg", img);
  // cv::imwrite("/home/sun/tof_edge.jpg", image_tof_edge_);

  //		cv::Ptr<cv::FeatureDetector> detector=cv::ORB::create();
  //		std::vector<cv::KeyPoint> key_points;
  //		detector->detect(img,key_points);
  //
  //		cv::Mat img_feature;
  //		cv::drawKeypoints(img,key_points,img_feature,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DEFAULT);
  //
  //		cv::namedWindow("tof_int_feature", cv::WINDOW_NORMAL);
  //		cv::imshow("tof_int_feature",img_feature);

  //		cv::waitKey(1);
}

void MsgSubscriber::callbackTofCloud(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {

  buff_mutex_.lock();

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *point_cloud);
  deque_point_cloud_tof_.push_back(point_cloud);
  deque_timestamp_tof_cloud_.push_back(msg->header.stamp.toSec());

  buff_mutex_.unlock();

  // flag_tof_cloud_received_ = true;
  // timestamp_tof_cloud_ = msg->header.stamp.toSec();
}

void MsgSubscriber::callbackIR(const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (!ptr_img->image.empty()) {
    deque_image_ir_.push_back(ptr_img->image);
    deque_timestamp_ir_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}

void MsgSubscriber::callbackMultispec(const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_multispec_.push_back(ptr_img->image);
    deque_timestamp_multispec_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}

void MsgSubscriber::callbackMultispec2(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_multispec_2_.push_back(ptr_img->image);
    deque_timestamp_multispec_2_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}

void MsgSubscriber::callbackMultispec3(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_multispec_3_.push_back(ptr_img->image);
    deque_timestamp_multispec_3_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}

void MsgSubscriber::callbackMultispec4(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_multispec_4_.push_back(ptr_img->image);
    deque_timestamp_multispec_4_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}

void MsgSubscriber::callbackMultispec5(
    const sensor_msgs::Image::ConstPtr &msg) {

  buff_mutex_.lock();

  cv_bridge::CvImageConstPtr ptr_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  if (!ptr_img->image.empty()) {
    deque_image_multispec_5_.push_back(ptr_img->image);
    deque_timestamp_multispec_5_.push_back(msg->header.stamp.toSec());
  }

  buff_mutex_.unlock();
}
} // namespace ZJL
