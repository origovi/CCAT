/**
 * @file Matcher.cpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Matcher module
 * @version 1.0
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#include "modules/Matcher.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

void Matcher::copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output) {
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    output[i] = pcl::make_shared<PCL>(*input[i]);
  }
}

void Matcher::projections(const std::vector<Observation::Ptr> &observations, std::vector<Projection> &projs) {
  projs.reserve(observations.size());
  Projection proj;
  //std::cout << '[' << which << "]: Recons Size: " << recons.size() << std::endl;

  for (size_t i = 0; i < observations.size(); i++) {
    proj.observation = observations[i];
    bool hasAnyPointInImage = false;

    proj.projPoints.resize(observations[i]->pcl->size());

    for (size_t pInd = 0; pInd < observations[i]->pcl->size(); pInd++) {
      // Convert point to image coordinates (axis must be reassigned)
      Eigen::Vector3f aux = observations[i]->pcl->operator[](pInd).getVector3fMap();
      aux = Eigen::Vector3f(-aux.y(), -aux.z(), aux.x());
      
      // Transform point with camera intrinsics
      Eigen::Vector3f transformedP(intrinsics_ * aux);
      
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
      Eigen::Vector3f aux = observations[i]->centroid_transformed.vec3f();
      aux = Eigen::Vector3f(-aux.y(), -aux.z(), aux.x());
      Eigen::Vector3f imageCentroidVec(intrinsics_ * aux);
    
      proj.imageCentroid.x = imageCentroidVec.x() / imageCentroidVec.z();
      proj.imageCentroid.y = imageCentroidVec.y() / imageCentroidVec.z();
      
      projs.push_back(proj);
    } else {
      currentCones_.emplace_back(observations[i]);
    }
  }
  std::cout << '[' << which << "]: Projs Size: " << projs.size() << std::endl;
}

void Matcher::publishPCLs(const std::vector<Observation::Ptr> &observations) const {
  PCL pclToPaint;
  for (const Observation::Ptr &obs : observations) {
    pclToPaint += *obs->pcl;
  }
  sensor_msgs::PointCloud2 pcl;
  pcl::toROSMsg(pclToPaint, pcl);
  pcl.header.frame_id = "map";
  pcl.header.stamp = ros::Time::now();
  pclPub_.publish(pcl);
}

void Matcher::publishImage(const std::vector<Projection> &projections,
                           const geometry_msgs::PoseArray::ConstPtr &bbs) const {
  cv::Mat image(params_.image_height, params_.image_width, CV_8UC3, cv::Scalar(176, 176, 176));

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
      cv::circle(image, p, 2, CV_RGB(255, 0, 0), -1);
    }
    cv::circle(image, proj.imageCentroid, 4, CV_RGB(0, 255, 0), -1);
  }

  // The timestamp will be taken from the bbs message header
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(bbs->header, "bgr8", image).toImageMsg();
  projectedPub_.publish(msg);
}

double Matcher::bbHeightFromZ(const double &z) const {
  //return 0.0;
  // Eigen::Vector3f aux = intrinsics_ * Eigen::Vector3f(dist, 0.0, params_.cone_height);
  // return double(aux.y() / aux.z());
  Eigen::Vector3f top = intrinsics_ * Eigen::Vector3f(0.0, params_.cone_height, z);
  //std::cout << "Top: " << top.x() << " " << top.y() << " " << top.z() << std::endl;
  Eigen::Vector3f bottom = intrinsics_ * Eigen::Vector3f(0.0, 0.0, z);
  // std::cout << "Bottom: " << bottom.x() << " " << bottom.y() << " " << bottom.z() << std::endl;
  return double((top.y() / top.z()) - (bottom.y() / bottom.z()));
}

Point Matcher::bbCentroidAndHeight(const geometry_msgs::Pose &bb) {
  return Point((bb.orientation.x + bb.orientation.z) / 2, (bb.orientation.y + bb.orientation.w) / 2, abs(bb.orientation.y - bb.orientation.w));
  //return Point((bb.orientation.x + bb.orientation.z) / 2, (bb.orientation.y + bb.orientation.w) / 2, 0.0);
}

void Matcher::matchBestFit(const size_t &bbInd, const geometry_msgs::PoseArray &bbs, const KDTree &projsKDT, std::vector<Matching> &matches, std::vector<std::set<size_t>> &projsToExclude) const {
  Point bbCentroidAndHeightP(bbCentroidAndHeight(bbs.poses[bbInd]));
  bool isFirst = true;
  pointIndexV projPointInd;
  projPointInd = projsKDT.nearest_pointIndex(bbCentroidAndHeightP, projsToExclude[bbInd]);
  double dist = Point::dist(bbCentroidAndHeightP, projPointInd->first);

  while (ros::ok() and (isFirst or dist >= matches[projPointInd->second].dist())) {
    // No matching is possible
    if (!bool(projPointInd) or dist > params_.max_match_search_dist) {
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

      matchBestFit(bbIndToRematch, bbs, projsKDT, matches, projsToExclude);

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

void Matcher::matchGreedy(const size_t &projInd, const std::vector<Projection> &projections, const KDTree &bbsKDT, std::vector<Matching> &matches) const {
  Point projCentroidAndBbHeight(projections[projInd].imageCentroid.x, projections[projInd].imageCentroid.y, bbHeightFromZ(projections[projInd].observation->centroid_transformed.x));
  pointIndexV bbPointInd(bbsKDT.nearest_pointIndex(projCentroidAndBbHeight));
  double dist = Point::dist(projCentroidAndBbHeight, bbPointInd->first);
  std::cout << "bb height: " << projCentroidAndBbHeight.z << " " << bbPointInd->first.z << std::endl;
  // No matching is possible
  if (!bool(bbPointInd) or dist > params_.max_match_search_dist) {
    return;
  }

  // Match the projection with the bb
  else {
    matches[projInd].match(bbPointInd->second, dist);
  }
}

void Matcher::computeMatches(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs) {
  std::vector<Matching> matches(projections.size());

  /* Two types of matching search */

  // Best Fit search:
  // We will match each bounding box to the closest (with a best fit),
  // that means that each bounding box will be matched only once.
  if (params_.match_type == "best fit") {
    std::vector<std::set<size_t>> projsToExclude(bbs.poses.size());

    std::vector<Point> pointsToBuildKDTree;
    pointsToBuildKDTree.reserve(projections.size());
    for (const Projection &proj : projections) {
      pointsToBuildKDTree.emplace_back(proj.imageCentroid.x, proj.imageCentroid.y, bbHeightFromZ(proj.observation->centroid_transformed.x));
    }
    KDTree projsKDT(pointsToBuildKDTree);
    for (size_t bbInd = 0; bbInd < bbs.poses.size(); bbInd++) {
      matchBestFit(bbInd, bbs, projsKDT, matches, projsToExclude);
    }
  }

  // Greedy search:
  // We will match each projection to the closest bounding box
  else {
    // Each Point will have X and Y (image coordinates) and the Z will be the
    // height (in pixels) that the BB of the cone would have.
    std::vector<Point> pointsToBuildKDTree;
    pointsToBuildKDTree.reserve(bbs.poses.size());
    for (const geometry_msgs::Pose &bb : bbs.poses) {
      pointsToBuildKDTree.push_back(bbCentroidAndHeight(bb));
    }
    KDTree bbsKDT(pointsToBuildKDTree);

    for (size_t projInd = 0; projInd < projections.size(); projInd++) {
      matchGreedy(projInd, projections, bbsKDT, matches);
    }
  }

  // Create the cones:
  // 1. Every SLAM-seen Observation will have the equivalent Cone
  // 2. The Cone will have the type of the BB that it's matched to
  // 3. TODO: if any Observation is very close and does not have a match,
  //    delete it
  size_t matches_num = 0;
  for (size_t projInd = 0; projInd < matches.size(); projInd++) {
    // Create a cone from this observation giving it the matching distance
    currentCones_.emplace_back(projections[projInd].observation, matches[projInd].dist());

    // If there is a match for this observation and it fulfills the conditions to classify the cone:
    // 1. The distance from the cone to the car is below a maximum
    if (bool(matches[projInd])/* and projections[projInd].observation->distToCar <= params_.max_match_real_dist*/) {
      matches_num++;
      currentCones_.back().setTypeFromAsMsgs(bbs.poses[matches[projInd].bbInd()].position.x);
    }
  }
  std::cout << '[' << which << "]: Matchings Size: " << matches_num << std::endl;

  // Calibrate this camera
  if (!calibrated_) {
    ccat::CalibReq req;
    req.request.euler_angles.resize(3);
    Eigen::Vector3d::Map(&req.request.euler_angles[0], 3) = extrinsics_.rotation().eulerAngles(0, 1, 2);
    req.request.translation.resize(3);
    Eigen::Vector3d::Map(&req.request.translation[0], 3) = Eigen::Vector3d(extrinsics_.translation().col(0).head(3));
    req.request.camera_matrix = std::vector<float>(intrinsics_.data(), intrinsics_.data() + intrinsics_.size());
    req.request.bbsCentroids.reserve(matches_num);
    req.request.obsCentroids.reserve(matches_num);
    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(matches.size());
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = "map";
    m.pose.orientation.w = 1.0;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.lifetime = ros::Duration(0.13);
    int id = 0;
    for (size_t projInd = 0; projInd < matches.size(); projInd++) {
      m.id = id++;
      if (bool(matches[projInd])) {
        req.request.obsCentroids.push_back(projections[projInd].observation->centroid_base_link.gmPoint());
        req.request.bbsCentroids.push_back(bbCentroidAndHeight(bbs.poses[matches[projInd].bbInd()]).gmPoint());
        m.pose.position = projections[projInd].observation->centroid_base_link.gmPoint();
        ma.markers.push_back(m);
      }
    }
    nose_.publish(ma);
    if (calibSrv_.call(req)) {
      // Udate the extrinsics
      Eigen::Quaterniond rotationQ(
        Eigen::AngleAxisd(req.response.euler_angles[0], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(req.response.euler_angles[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(req.response.euler_angles[2], Eigen::Vector3d::UnitZ()));
      extrinsics_.linear() = rotationQ.toRotationMatrix();
      extrinsics_.translation() = Eigen::Vector3d(req.response.translation.data());
      //calibrated_ = true;
      ROS_INFO("[ccat]: Matcher calibrated succesfully!");
    } else {
      ROS_ERROR("[ccat]: Failed to call Calibration Service");
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

Matcher::Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, const Which &which) : which(which), params_(params), intrinsics_(params.intrinsics.data()) {
  hasValidData_ = false;
  calibrated_ = false;

  // Extrinsics
  Eigen::Quaterniond rotationQ(
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[2], Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(params_.extrinsics.translation.data());

  // Subscribers declaration
  calibSrv_ = nh->serviceClient<ccat::CalibReq>("/AS/P/calibration");

  // Publishers declaration
  if (params_.debug) {
    std::string noseCachopo = ((which == LEFT) ? "left" : "right");
    nose_ = nh->advertise<visualization_msgs::MarkerArray>("/AS/P/ccat/nose/" + noseCachopo, 1);
    image_transport::ImageTransport image_transport(*nh);
    projectedPub_ = image_transport.advertise(params_.topics.output.projected, 1);
    pclPub_ = nh->advertise<sensor_msgs::PointCloud2>(params_.topics.output.pcl, 1);
  }
}

void Matcher::cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level) {
  ROS_WARN("cfg callback");
  Eigen::Quaterniond rotationQ(Eigen::AngleAxisd(config.roll, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(config.pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(config.yaw, Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(config.x, config.y, config.z);
}

void Matcher::run(const RqdData &data) {
  if (calibrated_) hasValidData_ = true;
  std::vector<Observation::Ptr> observations(data.observations.size());
  cvrs::obs2ObsPtr(data.observations, observations);

  // Transform the observations (pcls AND centroid) to camera space
  for (Observation::Ptr &obs : observations) {
    pcl::transformPointCloud(*obs->pcl, *obs->pcl, extrinsics_);
    obs->centroid_transformed = obs->centroid_base_link.transformed(extrinsics_);
  }

  // Filter the points so only the points in front of the camera (visibles)
  // are left. Here we are removing the points that have (z < 0.0).
  pcl::ConditionalRemoval<PCLPoint> zFilter;
  pcl::ConditionAnd<PCLPoint>::Ptr zFilterCond(pcl::make_shared<pcl::ConditionAnd<PCLPoint>>());
  zFilterCond->addComparison(pcl::make_shared<pcl::FieldComparison<PCLPoint>>("x", pcl::ComparisonOps::GE, 0.0));
  zFilter.setCondition(zFilterCond);
  for (Observation::Ptr &obs : observations) {
    zFilter.setInputCloud(obs->pcl);
    zFilter.filter(*obs->pcl);
  }

  // If debug, publish all the cones in camera coordinates
  if (params_.debug) publishPCLs(observations);

  // Create the projections of the points, that is:
  // 1. Transform all the points to image space
  // 2. Save only the observations which have at least one point on the image
  // 3. Save to currentCones_ the cones that do not appear in the image (behind the car)
  std::vector<Projection> projs;
  currentCones_.clear();
  currentCones_.reserve(observations.size());
  projections(observations, projs);

  // If debug, publish the images showing the projections (aka calibration)
  if (params_.debug) publishImage(projs, data.bbs);

  // Now obtain the matches and save the matched observations
  computeMatches(projs, *data.bbs);
}

const std::vector<Cone> &Matcher::getData() const { return currentCones_; }

const bool &Matcher::hasValidData() const { return hasValidData_; }