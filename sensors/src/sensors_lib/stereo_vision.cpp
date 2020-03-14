#include "sensors_lib/stereo_vision.h"

#include <cv_bridge/cv_bridge.h>

namespace sensors
{
StereoVision::StereoVision(ros::NodeHandle nh) : 
  nh_(nh),
  exact_sync_(ExactPolicy(10),
    sub_left_image_, sub_left_camera_info_,
    sub_right_image_, sub_right_camera_info_)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<sensors::StereoVisionParamsConfig>::CallbackType cb;
  cb = boost::bind(&StereoVision::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  int num_disparities;
  pnh.param("numDisparities", num_disparities, num_disparities);
  num_disparities_ = convertNumDisparities(num_disparities);
  int block_size;
  pnh.param("blockSize", block_size, block_size);
  block_size_ = convertBlockSize(block_size);
  pnh.param("maxDepth", max_depth_);

  sub_left_image_.subscribe(nh_, "kitti/camera_gray_left/image_raw", 1);
  sub_left_camera_info_.subscribe(nh_, "/kitti/camera_gray_left/camera_info", 1);
  sub_right_image_.subscribe(nh_, "/kitti/camera_gray_right/image_raw", 1);
  sub_right_camera_info_.subscribe(nh_, "/kitti/camera_gray_right/camera_info", 1);
  exact_sync_.registerCallback(boost::bind(&StereoVision::callback,
                                              this, _1, _2, _3, _4));

  pub_disparity_ = nh.advertise<DisparityImage>("/kitti/stereo/disparity", 1);
  pub_disparity_image_ = nh_.advertise<Image>("/kitti/stereo/disparity_image", 1);
  pub_points2_ = nh_.advertise<PointCloud2>("/kitti/stereo/pointcloud", 1);

  block_matcher_ = cv::StereoBM::create(num_disparities_, block_size_);
  sg_block_matcher_ = cv::StereoSGBM::create(1, 1, 10);

}

void StereoVision::callback(const ImageConstPtr& l_image_msg,
                      const CameraInfoConstPtr& l_info_msg,
                      const ImageConstPtr& r_image_msg,
                      const CameraInfoConstPtr& r_info_msg)
{


  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border   = block_size_ / 2;
  int left   = num_disparities_ + block_matcher_->getMinDisparity() + border - 1;
  int wtf = (block_matcher_->getMinDisparity() >= 0) ? border + block_matcher_->getMinDisparity() : std::max(border, -block_matcher_->getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  // TODO PASS MODEL_
  processDisparity(l_image, r_image, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  pub_disparity_.publish(disp_msg);

  // OPTIONAL FOR VIZ
  cv_bridge::CvImage cv_bridge_disparity_image;
  cv_bridge_disparity_image.image = disparity16_;
  cv_bridge_disparity_image.encoding = "mono16";
  cv_bridge_disparity_image.header.stamp = l_image_msg->header.stamp;
  pub_disparity_image_.publish(cv_bridge_disparity_image.toImageMsg());


  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  points_msg->header = disp_msg->header;
  // Calculate point cloud
  points_msg->height = disp_msg->image.height;
  points_msg->width  = disp_msg->image.width;
  points_msg->is_bigendian = false;
  points_msg->is_dense = false; // there may be invalid points
  // TODO PASS MODEL_
  processPoints2(*disp_msg, l_image, "mono8", *points_msg);

  // TODO DIE POINTS RGB
  pub_points2_.publish(points_msg);
  
}

void StereoVision::processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                              DisparityImage& disparity)
{

  // Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r.
  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;

  // Block matcher produces 16-bit signed (fixed point) disparity image
  block_matcher_->compute(left_rect, right_rect, disparity16_);
  //sg_block_matcher_->compute(left_rect, right_rect, disparity16_);

  // Fill in DisparityImage image data, converting to 32-bit float
  Image& dimage = disparity.image;
  dimage.height = disparity16_.rows;
  dimage.width = disparity16_.cols;
  dimage.encoding = image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  // We convert from fixed-point to float disparity and also adjust for any x-offset between
  // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  disparity16_.convertTo(dmat, dmat.type(), inv_dpp, -(model_.left().cx() - model_.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  // StereoVision parameters
  disparity.f = model_.right().fx();
  disparity.T = model_.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity.min_disparity = block_matcher_->getMinDisparity();
  disparity.max_disparity = block_matcher_->getMinDisparity() + num_disparities_ - 1;
  disparity.delta_d = inv_dpp;
}

void StereoVision::processPoints2(const DisparityImage& disparity,
                            const cv::Mat& color, const std::string& encoding,
                            PointCloud2& points)
{
  
  // Calculate dense point cloud
  const Image& dimage = disparity.image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, dense_points_, true);

  // Fill in sparse point cloud message
  points.height = dense_points_.rows;
  points.width  = dense_points_.cols;
  points.fields.resize (4);
  points.fields[0].name = "x";
  points.fields[0].offset = 0;
  points.fields[0].count = 1;
  points.fields[0].datatype = PointField::FLOAT32;
  points.fields[1].name = "y";
  points.fields[1].offset = 4;
  points.fields[1].count = 1;
  points.fields[1].datatype = PointField::FLOAT32;
  points.fields[2].name = "z";
  points.fields[2].offset = 8;
  points.fields[2].count = 1;
  points.fields[2].datatype = PointField::FLOAT32;
  points.fields[3].name = "rgb";
  points.fields[3].offset = 12;
  points.fields[3].count = 1;
  points.fields[3].datatype = PointField::FLOAT32;
  //points.is_bigendian = false; ???
  points.point_step = 16;
  points.row_step = points.point_step * points.width;
  points.data.resize (points.row_step * points.height);
  points.is_dense = false; // there may be invalid points
 
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int i = 0;
  for (int32_t u = 0; u < dense_points_.rows; ++u) {
    for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
      if (isValidPoint(dense_points_(u,v))) {
        // x,y,z,rgba
        memcpy (&points.data[i * points.point_step + 0], &dense_points_(u,v)[0], sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &dense_points_(u,v)[1], sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &dense_points_(u,v)[2], sizeof (float));
      }
      else {
        memcpy (&points.data[i * points.point_step + 0], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &bad_point, sizeof (float));
      }
    }
  }

  // Fill in color
  namespace enc = image_encodings;
  i = 0;
  if (encoding == enc::MONO8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          memcpy (&points.data[i * points.point_step + 12], &rgb, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else {
    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
  
}

bool StereoVision::isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]) && pt[2] < max_depth_;
}

void StereoVision::configCallback(sensors::StereoVisionParamsConfig &config, uint32_t level __attribute__((unused)))
{
  num_disparities_ = convertNumDisparities(config.numDisparities);
  block_size_ = convertBlockSize(config.blockSize);
  max_depth_ = config.maxDepth;
  ROS_DEBUG("Reconfigure Request");
  ROS_DEBUG("numDisparities %d", num_disparities_);
  ROS_DEBUG("blockSize %d", block_size_);
  ROS_DEBUG("maxDepth %f", max_depth_);
  block_matcher_ = cv::StereoBM::create(num_disparities_, block_size_);
}

int StereoVision::convertNumDisparities(const int num_disparities){
  return 16 * num_disparities;
}

int StereoVision::convertBlockSize(const int block_size){
  return 2 * block_size + 5;
}

}
