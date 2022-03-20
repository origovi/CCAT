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

void Matcher::projections(const std::vector<PCL::Ptr> &recons, const std::vector<Observation::Ptr> &observations, std::vector<Projection> &projs) const {
  projs.reserve(recons.size());
  Projection proj;
  //std::cout << '[' << which << "]: Recons Size: " << recons.size() << std::endl;

  for (size_t i = 0; i < recons.size(); i++) {
    proj.observation = observations[i];
    bool hasAnyPointInImage = false;

    proj.projPoints.resize(recons[i]->size());

    for (size_t pInd = 0; pInd < recons[i]->size(); pInd++) {
      // Transform point with camera intrinsics
      Eigen::Vector3f transformedP(intrinsics_ * recons[i]->operator[](pInd).getVector3fMap());

      // Convert point to image coordinates (axis must be reassigned)
      proj.projPoints[pInd].x = transformedP.x() / transformedP.z();
      proj.projPoints[pInd].y = transformedP.y() / transformedP.z();

      // Check if point is inside the image, if so, this cone will be pushed
      // into the list containing all projected observations
      if (proj.projPoints[pInd].x > 0 and
          proj.projPoints[pInd].x < params_.image_width and
          proj.projPoints[pInd].y > 0 and
          proj.projPoints[pInd].y < params_.image_height) {
        hasAnyPointInImage = true;
      }
    }
    if (hasAnyPointInImage) {
      // 1. Transform the centroid of the pcl to image coords
      // 2. Push the projection to the result
      Eigen::Vector3f imageCentroidVec(intrinsics_ * observations[i]->centroid.vec3f());
      proj.imageCentroid.x = imageCentroidVec.x() / imageCentroidVec.z();
      proj.imageCentroid.y = imageCentroidVec.y() / imageCentroidVec.z();
      projs.push_back(proj);
    }
  }
  std::cout << '[' << which << "]: Projs Size: " << projs.size() << std::endl;
}

void Matcher::publishPCLs(const std::vector<PCL::Ptr> &pcls) const {
  PCL pclToPaint;
  for (auto &pcl : pcls) {
    pclToPaint += *pcl;
  }
  sensor_msgs::PointCloud2 pcl;
  pcl::toROSMsg(pclToPaint, pcl);
  pcl.header.frame_id = "map";
  pcl.header.stamp = ros::Time::now();
  pclPub_.publish(pcl);
}

void Matcher::publishImage(const std::vector<Projection> &projections,
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

  // The timestamp will be taken from the bbs message header
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(bbs->header, "bgr8", image).toImageMsg();
  projectedPub_.publish(msg);
}

double Matcher::bbHeightFromDist(const double &dist) const {
  Eigen::Vector3f aux = intrinsics_ * Eigen::Vector3f(0.0, params_.cone_height, dist);
  return double(aux.y() / aux.z());
}

Point Matcher::bbCentroidAndHeight(const geometry_msgs::Pose &bb) {
  return Point((bb.position.x + bb.position.z) / 2, (bb.position.y + bb.orientation.w) / 2, abs(bb.position.y - bb.orientation.w));
}

void Matcher::match(const size_t &bbInd, const geometry_msgs::PoseArray &bbs, const KDTree &projsKDT, std::vector<Match> &matches, std::vector<std::set<size_t>> &projsToExclude) const {
  Point bbCentroidAndHeightP(bbCentroidAndHeight(bbs.poses[bbInd]));
  bool isFirst = true;
  pointIndexV projPointInd;
  projPointInd = projsKDT.nearest_pointIndex(bbCentroidAndHeightP, projsToExclude[bbInd]);
  double dist = Point::dist(bbCentroidAndHeightP, projPointInd->first);

  while (ros::ok() and (isFirst or dist >= matches[projPointInd->second].dist())) {
    // No matching is possible
    if (!bool(projPointInd) /* or dist > params_.max_match_dist */) {
      return;
    }

    // The found Projection has not any match yet
    if (!matches[projPointInd->second]) {
      matches[projPointInd->second].match(bbInd, dist);
      return;
    }

    // The found Projection with this BB is a better match than the one before
    else if (dist < matches[projPointInd->second].dist()) {
      projsToExclude[matches[projPointInd->second].bbInd()].insert(projPointInd->second);
      size_t bbIndToRematch = matches[projPointInd->second].bbInd();
      matches[projPointInd->second].unmatch();

      match(bbIndToRematch, bbs, projsKDT, matches, projsToExclude);

      // Update the new match AFTER the reassignment(s)
      matches[projPointInd->second].match(bbInd, dist);
      return;
    }
    // The found Projection is worse, we search another time
    // adding the found Projection to the exclude set
    else {
      projsToExclude[bbInd].insert(projPointInd->second);
      projPointInd = projsKDT.nearest_pointIndex(bbCentroidAndHeightP, projsToExclude[bbInd]);
      dist = Point::dist(bbCentroidAndHeightP, projPointInd->first);
    }
    isFirst = false;
  }
}

void Matcher::computeMatches(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs) {
  std::vector<Match> matches(projections.size());
  std::vector<std::set<size_t>> projsToExclude(bbs.poses.size());
  // Each Point will have X and Y (image coordinates) and the Z will be the
  // height (in pixels) that the BB of the cone would have.
  std::vector<Point> pointsToBuildTheKDTree;
  pointsToBuildTheKDTree.reserve(projections.size());
  for (const Projection &proj : projections) {
    pointsToBuildTheKDTree.emplace_back(proj.imageCentroid.x, proj.imageCentroid.y, bbHeightFromDist(Point::dist(proj.observation->centroid)));
  }
  KDTree projsKDT(pointsToBuildTheKDTree);
  ROS_WARN("control2");
  for (size_t bbInd = 0; bbInd < bbs.poses.size(); bbInd++) {
    match(bbInd, bbs, projsKDT, matches, projsToExclude);
  }

  ROS_WARN("control");

  // Create the cones:
  // 1. Every SLAM-seen Observation will have the equivalent Cone
  // 2. The Cone will have the type of the BB that it's matched to
  // 3. TODO: if any Observation is very close and does not have a match,
  //    delete it
  //currentCones_.clear();
  currentCones_.resize(projections.size());
  size_t matches_num = 0;
  for (size_t projInd = 0; projInd < matches.size(); projInd++) {
    currentCones_[projInd] = Cone(projections[projInd].observation, matches[projInd].dist());
    if (bool(matches[projInd])) {
      matches_num++;
      currentCones_[projInd].setTypeFromAsMsgs(bbs.poses[matches[projInd].bbInd()].position.x);
    }
  }
  std::cout << '[' << which << "]: Matchings Size: " << matches_num << std::endl;
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
  image_transport::ImageTransport image_transport(*nh_);
  if (params_.debug) projectedPub_ = image_transport.advertise(params_.topics.output.projected, 1);
  if (params_.debug) pclPub_ = nh_->advertise<sensor_msgs::PointCloud2>(params_.topics.output.pcl, 1);
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

  // If debug, publish all the cones in camera coordinates
  if (params_.debug) publishPCLs(pcls);

  // Create the projections of the points, that is:
  // 1. Transform all the points to image space
  // 2. Save only the observations which have at least one point on the image
  std::vector<Projection> projs;
  projections(pcls, data.observations, projs);

  // If debug, publish the images showing the projections (aka calibration)
  if (params_.debug) publishImage(projs, data.bbs);

  // Now obtain the matches
  computeMatches(projs, *data.bbs);
}

/* Getters */

const std::vector<Cone> &Matcher::getData() const { return currentCones_; }
