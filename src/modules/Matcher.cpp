#include "modules/Matcher.hpp"

/**
 * PRIVATE METHODS
 */
void Matcher::copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output) {
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    output[i] = pcl::make_shared<PCL>(*input[i]);
  }
}

void Matcher::cameraTransformPCLs(const std::vector<PCL::Ptr> &reconstructions,
                                  const Extrinsics &camTf) const {
  for (size_t i = 0; i < reconstructions.size(); i++) {
    pcl::transformPointCloud(*reconstructions[i], *reconstructions[i], camTf);
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

std::list<Matcher::Projection> Matcher::transformPCLs(
    const std::vector<PCL::Ptr> &pcls,
    const Eigen::Transform<double, 3, Eigen::Projective> &tf) const {
  std::list<Projection> res;
  Projection proj;
  PCL temp_pcl;
  for (size_t obsIndex = 0; obsIndex < pcls.size(); obsIndex++) {
    proj.obsIndex = obsIndex;
    bool hasAnyPointInImage = false;
    pcl::transformPointCloud(*pcls[obsIndex], temp_pcl, tf.matrix());
    proj.projPoints.clear();
    proj.projPoints.reserve(temp_pcl.size());
    for (size_t pIndex = 0; pIndex < temp_pcl.size(); pIndex++) {
      cv::Point2d imagePoint(temp_pcl[pIndex].y / temp_pcl[pIndex].z,
                             temp_pcl[pIndex].x / temp_pcl[pIndex].z);
      //std::cout << imagePoint.x << " " << imagePoint.y << std::endl;
      proj.projPoints.push_back(imagePoint);
      if (imagePoint.x > 0 and imagePoint.x < params_.image_width and imagePoint.y > 0 and
          imagePoint.y < params_.image_height) {
        hasAnyPointInImage = true;
      }
    }
    if (hasAnyPointInImage) {
      res.push_back(proj);
    }
  }
  return res;
}

cv::Point2d Matcher::projectPoint(const PCLPoint &pointToProject,
                                  const Intrinsics &intrinsics) const {
  // Reassign axes: camera frame has different axes (X pointing right, Y down
  // and Z forward)
  // Eigen::Vector3d p_corrected(-pointToProject.y, -pointToProject.z, pointToProject.x);
  // Eigen::Vector3d p_corrected(pointToProject.x, pointToProject.y, pointToProject.z);

  // Eigen::Vector3d coords2D(intrinsics * Eigen::Vector3d(pointToProject.x, pointToProject.y,
  // pointToProject.z));

  // std::cout << coords2D[0] << " " << coords2D[1] << " " << coords2D[2] << std::endl;
  // float U = intrinsics.fx * p_corrected.x + intrinsics.cx * p_corrected.z;
  // float V = intrinsics.fy * p_corrected.y + intrinsics.cy * p_corrected.z;
  // return cv::Point2d(coords2D[1]/coords2D[2], coords2D[0]/coords2D[2]);
}

void Matcher::projections(const std::vector<PCL::Ptr> &recons, std::list<Projection> &projs) const {
  Projection proj;
  for (size_t i = 0; i < recons.size(); i++) {
    proj.obsIndex = i;
    bool hasPointsInImage = false;
    for (PCL::const_iterator it = recons[i]->begin(); it != recons[i]->end(); it++) {
      // check if point is in the image
    }
    if (hasPointsInImage) projs.push_back(proj);
  }
}

void Matcher::publishImage2(const std::list<Projection> &projections,
                            const geometry_msgs::PoseArray::ConstPtr &bbs) const {
  cv::Mat image(params_.image_height, params_.image_width, CV_8UC3, cv::Scalar(255, 255, 255));

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
  for (const Projection &proj : projections) {
    for (const cv::Point2d &p : proj.projPoints) {
      cv::circle(image, p, 3, CV_RGB(255, 0, 0), -1);
    }
  }

  // The timestamp will be taken from the car location
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(bbs->header, "bgr8", image).toImageMsg();
  projectedPub_.publish(msg);
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
      // if it->z<0
      if (it->z < 0) cv::circle(image, projectPoint(*it, intrinsics), 3, CV_RGB(255, 0, 0), -1);
    }
  }

  // The timestamp will be taken from the car location
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(current_header_, "bgr8", image).toImageMsg();
  imPub.publish(msg);
}

void Matcher::match(const std::list<Projection> &projections, const geometry_msgs::PoseArray::ConstPtr &bbs, std::vector<Cone> &matchings) {
  matchings.clear();
}

/**
 * PUBLIC CONSTRUCTOR AND DESTRUCTOR
 */

Matcher::Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, const Which &which) : which(which), params_(params), intrinsics_(params.intrinsics.data()) {
  nh_ = nh;

  // Extrinsics
  Eigen::Quaterniond rotationQ(
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[2], Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(params_.extrinsics.translation.data());

  // Publishers declaration
  image_transport::ImageTransport image_transport(*nh);
  projectedPub_ = image_transport.advertise(params_.topics.output.projected, 1);
}

Matcher::~Matcher() {}

/**
 * PUBLIC METHODS
 */

/* Callbacks */

void Matcher::cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level) {
  ROS_WARN("cfg callback");
  Eigen::Quaterniond rotationQ(Eigen::AngleAxisd(config.roll, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(config.pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(config.yaw, Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(config.x, config.y, config.z);
}

void catxopo(const std::vector<PCL::Ptr> &recons, ros::NodeHandle &nh) {
  static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("locoL", 1);
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

void catxopo2(const std::vector<PCL::Ptr> &recons, ros::NodeHandle &nh) {
  static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("locoR", 1);
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

void Matcher::run(const RqdData &data) {
  std::vector<PCL::Ptr> pcls;
  cvrs::obs2PCLs(data.observations, pcls);

  // Transform the pointclouds to camera coordinates
  for (PCL::Ptr &pcl : pcls) {
    pcl::transformPointCloud(*pcl, *pcl, extrinsics_);
  }

  // Filter the points so only the points in front of the camera (visibles)
  // are left. Here we are removing the points that have (z < 0.0).
  pcl::ConditionalRemoval<PCLPoint> zFilter;
  pcl::ConditionAnd<PCLPoint>::Ptr zFilterCond(pcl::make_shared<pcl::ConditionAnd<PCLPoint>>());
  zFilterCond->addComparison(pcl::make_shared<pcl::FieldComparison<PCLPoint>>("z", pcl::ComparisonOps::GE, 0.0));
  zFilter.setCondition(zFilterCond);
  for (PCL::Ptr &pcl : pcls) {
    zFilter.setInputCloud(pcl);
    zFilter.filter(*pcl);
  }
  //std::cout << '[' << which << "]: " << "hola4" << std::endl;

  (which == Which::LEFT) ? catxopo(pcls, *nh_) : catxopo2(pcls, *nh_);

  std::list<Projection> projs;
  projections(pcls, projs);

  publishImage2(projs, data.bbs);

  // std::list<Projection> recons(transformPCLs(
  //     reconstructedPCLs(observations), intrinsics_left_ * extrinsics_left_ * extrinsics_car_));


  // match(recons, bbs_, currentCones_);
  // catxopo(reconsL, *nh_);
  // catxopo2(reconsR, *nh_);

  // std::vector<std::pair<>> publishImage(reconsL, leftBbs_, leftProjectedPub_, intrinsics_left_);
  // publishImage(reconsR, rightBbs_, rightProjectedPub_, intrinsics_right_);

  // match()
}

/* Getters */

const std::vector<Cone> &Matcher::getData() const { return currentCones_; }
