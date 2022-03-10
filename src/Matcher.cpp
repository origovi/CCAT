#include "Matcher.hpp"

/**
 * CONSTRUCTORS
 */
Matcher::Matcher() : hasData_(false) {}

/**
 * DESTRUCTORS
 */
Matcher::~Matcher() {}

/**
 * PRIVATE METHODS
 */
void Matcher::locationTransformPCLs(const std::vector<PCL::Ptr> &reconstructions) const {
  Eigen::Affine3d locationTransform;
  tf::poseMsgToEigen(carLocation_->pose.pose, locationTransform);
  for (size_t i = 0; i < reconstructions.size(); i++) {
    pcl::transformPointCloud(*actualMap_, *reconstructions[i], locationTransform);
  }
}

void Matcher::cameraTransformPCLs(const std::vector<PCL::Ptr> &reconstructions, const Eigen::Affine3d &camTf) const {
  for (size_t i = 0; i < reconstructions.size(); i++) {
    pcl::transformPointCloud(*actualMap_, *reconstructions[i], camTf);
  }
}

std::vector<PCL::Ptr> Matcher::reconstructedPCLs(const std::vector<Observation> &observations) const {
  std::vector<PCL::Ptr> res(observations.size());
  for (size_t i = 0; i < observations.size(); i++) {
    res[i] = observations[i].pcl;
  }
  return res;
}

cv::Point2d Matcher::projectPoint(const PCLPoint &pointToProject, const Params::Matcher::Intrinsics &intrinsics) const {
  // Reassign axes: camera frame has different axes (X pointing right, Y down and Z forward)
  Point p_corrected;
  p_corrected.x = -pointToProject.y;
  p_corrected.y = -pointToProject.z;
  p_corrected.z = pointToProject.x;

  float U = intrinsics.fx * p_corrected.x + intrinsics.cx * p_corrected.z;
  float V = intrinsics.fy * p_corrected.y + intrinsics.cy * p_corrected.z;
  return cv::Point2d(U / p_corrected.z, V / p_corrected.z);
}

void Matcher::publishImage(const std::vector<PCL::Ptr> &recons, const geometry_msgs::PoseArray::ConstPtr &bbs, const image_transport::Publisher &imPub, const Params::Matcher::Intrinsics &intrinsics) const {
  cv::Mat image(768, 1024, CV_8UC3, cv::Scalar(255, 255, 255));

  // Paint the bbs
  for (size_t i = 0; i < bbs->poses.size(); i++) {
    cv::Point2d pt1(bbs->poses[i].orientation.x, bbs->poses[i].orientation.y);
    cv::Point2d pt2(bbs->poses[i].orientation.z, bbs->poses[i].orientation.w);
    if (bbs->poses[i].position.x == 0)
      cv::rectangle(image, pt1, pt2, cv::Scalar(50, 255, 255), 2, 8, 0);
    else if (bbs->poses[i].position.x == 1)
      cv::rectangle(image, pt1, pt2, cv::Scalar(255, 0, 0), 2, 8, 0);
    else
      cv::rectangle(image, pt1, pt2, cv::Scalar(0, 145, 255), 2, 8, 0);
  }

  // Paint the points
  for (size_t i = 0; i < recons.size(); i++) {
    for (PCL::const_iterator it = recons[i]->begin(); it != recons[i]->end(); it++) {
      cv::circle(image, projectPoint(*it, intrinsics), 3, CV_RGB(255, 0, 0), -1);
    }
  }

  // The timestamp will be taken from the car location
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(carLocation_->header, "bgr8", image).toImageMsg();
  imPub.publish(msg);
}

/**
 * PUBLIC METHODS
 */
/* Singleton pattern */
Matcher &Matcher::getInstance() {
  static Matcher matcher;
  return matcher;
}

/* Init */
void Matcher::init(ros::NodeHandle *const &nh, const Params::Matcher &params) {
  nh_ = nh;
  params_ = params;

  // Publishers declaration
  image_transport::ImageTransport image_transport(*nh);
  leftProjectedPub_ = image_transport.advertise("/leftImage", 1);
  rightProjectedPub_ = image_transport.advertise("/rightImage", 1);
}

/* Callbacks */
void Matcher::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  if (cloud_msg->data.empty())
    ROS_WARN("Trying to read empty pointcloud");
  else
    pcl::fromROSMsg(*cloud_msg, *actualMap_);
}

void Matcher::locationAndBbsCbk(const nav_msgs::Odometry::ConstPtr &carPos, const geometry_msgs::PoseArray::ConstPtr &leftDetections, const geometry_msgs::PoseArray::ConstPtr &rightDetections) {
  carLocation_ = carPos;
  leftBbs_ = leftDetections;
  rightBbs_ = rightDetections;
  hasData_ = true;
}

/* Functions */
void Matcher::run(const std::vector<Observation> &observations) {
  std::vector<PCL::Ptr> reconsL = reconstructedPCLs(observations);
  locationTransformPCLs(reconsL);

  // Deep copy the reconstructions
  std::vector<PCL::Ptr> reconsR(reconsL.size());
  for (size_t i = 0; i < reconsL.size(); i++) {
    reconsR[i] = pcl::make_shared<PCL>(*reconsL[i]);
  }

  cameraTransformPCLs(reconsL, params_.tf_left);
  cameraTransformPCLs(reconsR, params_.tf_right);

  publishImage(reconsL, leftBbs_, leftProjectedPub_, params_.intrinsics_left);
  publishImage(reconsR, rightBbs_, rightProjectedPub_, params_.intrinsics_right);
}

/* Getters */
const bool &Matcher::hasData() const {
  return hasData_;
}
