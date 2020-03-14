#ifndef STEREOVISION_H
#define STEREOVISION_H

#include "sensors/StereoVisionParamsConfig.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_geometry/stereo_camera_model.h>

namespace sensors
{

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class StereoVision
{
 public:
  //! Constructor.
  explicit StereoVision(ros::NodeHandle nh);

 private:
  void configCallback(sensors::StereoVisionParamsConfig &config, uint32_t level);

  void callback(const ImageConstPtr& l_image_msg,
                const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& r_image_msg,
                const CameraInfoConstPtr& r_info_msg);

  void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                        DisparityImage& disparity);
  void processPoints2(const DisparityImage& disparity,
                      const cv::Mat& color, const std::string& encoding,
                      PointCloud2& points);
  bool isValidPoint(const cv::Vec3f& pt);
  int convertNumDisparities(const int num_disparities);
  int convertBlockSize(const int block_size);
  
  ros::NodeHandle nh_;

  message_filters::Subscriber<Image> sub_left_image_;
  message_filters::Subscriber<CameraInfo> sub_left_camera_info_;
  message_filters::Subscriber<Image> sub_right_image_;
  message_filters::Subscriber<CameraInfo> sub_right_camera_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  ExactSync exact_sync_;

  ros::Publisher pub_disparity_;
  ros::Publisher pub_disparity_image_;;
  ros::Publisher pub_points2_;

  dynamic_reconfigure::Server<sensors::StereoVisionParamsConfig> dr_srv_;

  image_geometry::StereoCameraModel model_;
  cv::Ptr<cv::StereoBM> block_matcher_;
  cv::Ptr<cv::StereoSGBM> sg_block_matcher_;

  cv::Mat_<int16_t> disparity16_;
  cv::Mat_<cv::Vec3f> dense_points_;

  int num_disparities_;
  int block_size_;
  float max_depth_;
};
}

#endif  // STEREOVISION_H
