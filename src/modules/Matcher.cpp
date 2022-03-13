#include "modules/Matcher.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */
Matcher::Matcher() : hasData_(false) {}
Matcher::~Matcher() {}

/**
 * PRIVATE METHODS
 */
void Matcher::copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output) {
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    output[i] = pcl::make_shared<PCL>(*input[i]);
  }
}

void Matcher::locationTransformPCLs(const std::vector<PCL::Ptr> &reconstructions) const {
  Eigen::Affine3d locationTransform;
  tf::poseMsgToEigen(carLocation_->pose.pose, locationTransform);
  for (size_t i = 0; i < reconstructions.size(); i++) {
    pcl::transformPointCloud(*reconstructions[i], *reconstructions[i], locationTransform.inverse());
  }
}

void Matcher::cameraTransformPCLs(const std::vector<PCL::Ptr> &reconstructions,
                                  const Extrinsics &camTf) const {
  for (size_t i = 0; i < reconstructions.size(); i++) {
    pcl::transformPointCloud(*reconstructions[i], *reconstructions[i], camTf.inverse());
  }
}

std::vector<PCL::Ptr> Matcher::reconstructedPCLs(
    const std::vector<Observation> &observations) const {
  std::vector<PCL::Ptr> res(observations.size());
  for (size_t i = 0; i < observations.size(); i++) {
    res[i] = observations[i].pcl;
  }
  return res;
}

cv::Point2d Matcher::projectPoint(const PCLPoint &pointToProject,
                                  const Intrinsics &intrinsics) const {
  // Reassign axes: camera frame has different axes (X pointing right, Y down
  // and Z forward)
  Eigen::Vector3d p_corrected(-pointToProject.y, -pointToProject.z, pointToProject.x);
  //Eigen::Vector3d p_corrected(pointToProject.x, pointToProject.y, pointToProject.z);

  Eigen::Vector3d coords2D(intrinsics*p_corrected);
  //std::cout << coords2D[0] << " " << coords2D[1] << std::endl;
  // float U = intrinsics.fx * p_corrected.x + intrinsics.cx * p_corrected.z;
  // float V = intrinsics.fy * p_corrected.y + intrinsics.cy * p_corrected.z;
  return cv::Point2d(coords2D[0], coords2D[1]);
}

void Matcher::publishImage(const std::vector<PCL::Ptr> &recons,
                           const geometry_msgs::PoseArray::ConstPtr &bbs,
                           const image_transport::Publisher &imPub,
                           const Intrinsics &intrinsics) const {
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

  // Left Tf
  Eigen::Quaterniond leftRotation(
      Eigen::AngleAxisd(params_.extrinsics_left.euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.extrinsics_left.euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.extrinsics_left.euler_angles[2], Eigen::Vector3d::UnitZ()));
  extrinsics_left_.linear() = leftRotation.toRotationMatrix();
  extrinsics_left_.translation() = Eigen::Vector3d(params_.extrinsics_left.translation.data());

  // Right Tf
  Eigen::Quaterniond rightRotation(
      Eigen::AngleAxisd(params_.extrinsics_right.euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.extrinsics_right.euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.extrinsics_right.euler_angles[2], Eigen::Vector3d::UnitZ()));
  extrinsics_right_.linear() = rightRotation.toRotationMatrix();
  extrinsics_right_.translation() = Eigen::Vector3d(params_.extrinsics_right.translation.data());

  intrinsics_left_ = Intrinsics(params_.intrinsics_left.data()).transpose();
  intrinsics_right_ = Intrinsics(params_.intrinsics_right.data()).transpose();

  // Publishers declaration
  image_transport::ImageTransport image_transport(*nh);
  leftProjectedPub_ = image_transport.advertise(params_.topics.output.projected_left, 1);
  rightProjectedPub_ = image_transport.advertise(params_.topics.output.projected_right, 1);
}

/* Callbacks */
void Matcher::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  if (cloud_msg->data.empty())
    ROS_WARN("Trying to read empty pointcloud");
  else
    pcl::fromROSMsg(*cloud_msg, *actualMap_);
}

void Matcher::locationAndBbsCbk(const nav_msgs::Odometry::ConstPtr &carPos,
                                const geometry_msgs::PoseArray::ConstPtr &leftDetections,
                                const geometry_msgs::PoseArray::ConstPtr &rightDetections) {
  carLocation_ = carPos;
  leftBbs_ = leftDetections;
  rightBbs_ = rightDetections;
  hasData_ = true;
  ROS_WARN("location and bbs callback");
}

void catxopo(const std::vector<PCL::Ptr> &recons, ros::NodeHandle &nh) {
  static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("loco", 1);
  PCL p;
  for (auto &nose : recons) {
    p += *nose;
  }
  sensor_msgs::PointCloud2 pcl;
  pcl::toROSMsg(p, pcl);
  pcl.header.frame_id = "map";
  pcl.header.stamp = ros::Time::now();
  pub.publish(pcl);
}

/* Functions */
void Matcher::run(const std::vector<Observation> &observations) {
  std::vector<PCL::Ptr> reconsL = reconstructedPCLs(observations);
  copyPCLPtrVec(reconsL, reconsL);
  locationTransformPCLs(reconsL);

  // Deep copy the reconstructions
  std::vector<PCL::Ptr> reconsR;
  copyPCLPtrVec(reconsL, reconsR);

  cameraTransformPCLs(reconsL, extrinsics_left_);
  cameraTransformPCLs(reconsR, extrinsics_right_);
  catxopo(reconsR, *nh_);


  publishImage(reconsL, leftBbs_, leftProjectedPub_, intrinsics_left_);
  publishImage(reconsR, rightBbs_, rightProjectedPub_, intrinsics_right_);
}

/* Getters */
const bool &Matcher::hasData() const { return hasData_; }

const std::vector<Cone> &Matcher::getCurrentCones() const {
  return currentCones_;
}
