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

#include "modules/Matcher/Matcher.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

void Matcher::projections(const std::vector<ProjectionData> &projDatas, std::vector<Projection> &projs) {
  projs.reserve(projDatas.size());
  Projection proj;
  //std::cout << '[' << which << "]: Recons Size: " << recons.size() << std::endl;

  for (size_t i = 0; i < projDatas.size(); i++) {
    proj.data = &projDatas[i];
    bool hasAnyPointInImage = false;

    proj.projPoints.resize(projDatas[i].pcl_cam->size());

    for (size_t pInd = 0; pInd < projDatas[i].pcl_cam->size(); pInd++) {
      // Convert point to image coordinates (axis must be reassigned)
      Eigen::Vector3f aux = projDatas[i].pcl_cam->operator[](pInd).getVector3fMap();
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
      Eigen::Vector3f aux = projDatas[i].centroid_cam.vec3f();
      aux = Eigen::Vector3f(-aux.y(), -aux.z(), aux.x());
      Eigen::Vector3f imageCentroidVec(intrinsics_ * aux);

      proj.imageCentroid.x = imageCentroidVec.x() / imageCentroidVec.z();
      proj.imageCentroid.y = imageCentroidVec.y() / imageCentroidVec.z();

      projs.push_back(proj);
    }
  }
  std::cout << '[' << which << "]: Projs Size: " << projs.size() << std::endl;
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
  Point projCentroidAndBBHeight(projections[projInd].imageCentroid.x, projections[projInd].imageCentroid.y, bbHeightFromZ(projections[projInd].data->centroid_cam.x));
  pointIndexV bbPointInd(bbsKDT.nearest_pointIndex(projCentroidAndBBHeight));
  double dist = Point::dist(projCentroidAndBBHeight, bbPointInd->first);
  std::cout << "bb height: " << projCentroidAndBBHeight.z << " " << bbPointInd->first.z << std::endl;
  // No matching is possible
  if (!bool(bbPointInd) or dist > params_.max_match_search_dist) {
    return;
  }

  // Match the projection with the bb
  else {
    matches[projInd].match(bbPointInd->second, dist);
  }
}

void Matcher::computeMatchings(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs, std::vector<Matching> &matchings) {
  matchings.resize(projections.size());

  /* Two types of matching search */

  // Best Fit search:
  // We will match each bounding box to the closest (with a best fit),
  // that means that each bounding box will be matched only once.
  if (params_.match_type == "best fit") {
    std::vector<std::set<size_t>> projsToExclude(bbs.poses.size());

    std::vector<Point> pointsToBuildKDTree;
    pointsToBuildKDTree.reserve(projections.size());
    for (const Projection &proj : projections) {
      pointsToBuildKDTree.emplace_back(proj.imageCentroid.x, proj.imageCentroid.y, bbHeightFromZ(proj.data->centroid_cam.x));
    }
    KDTree projsKDT(pointsToBuildKDTree);
    for (size_t bbInd = 0; bbInd < bbs.poses.size(); bbInd++) {
      matchBestFit(bbInd, bbs, projsKDT, matchings, projsToExclude);
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
      matchGreedy(projInd, projections, bbsKDT, matchings);
    }
  }
}

void Matcher::updateData(const std::vector<Projection> &projs, const std::vector<ProjectionData> &projDatas, const geometry_msgs::PoseArray &bbs, const std::vector<Matching> &matchings) {
  currentUpdates_.clear();
  currentUpdates_.reserve(projDatas.size());
  size_t matchings_num = 0;

  // Create the ConeUpdate(s):
  // 1. Every camera-seen Observation will be update through a ConeUpdate
  // 2. The ConeUpdate will have the type of the BB that it's matched to
  // 3. If the ConeUpdate Observation does not have any matching, None type will
  //    be assigned, and distToClosestMatch will be included.
  std::vector<Point> matchedCentroids;
  std::vector<bool> matchedObservations(projDatas.size(), false);
  matchedCentroids.reserve(matchings.size());
  // Add the ConeUpdate(s) that have a valid matching to the currentUpdates_
  // attribute, also add the centroids to a vector so then we can get
  // the closest matched observation for every non-matched one.
  for (size_t i = 0; i < matchings.size(); i++) {
    if (bool(matchings[i])) {
      matchedObservations[projs[i].data->obsInd] = true;
      matchings_num++;
      currentUpdates_.emplace_back(projs[i].data->obs->id, bbs.poses[matchings[i].bbInd()].position.x, matchings[i].dist(), projs[i].data->centroid_cam.x);
      matchedCentroids.push_back(projs[i].data->obs->centroid_global);
    }
  }
  // Create a KDTree with (only) the matched observation centroids, and for
  // every non-matched observation
  KDTree obsWithMatchKDT(matchedCentroids);
  for (size_t i = 0; i < projDatas.size(); i++) {
    if (!matchedObservations[i]) {
      KDTData<size_t> closestMatchingInd = obsWithMatchKDT.nearest_index(projDatas[i].obs->centroid_global);
      if (bool(closestMatchingInd)) {
        currentUpdates_.emplace_back(projDatas[i].obs->id, projDatas[i].centroid_cam.x, Point::dist(projDatas[i].obs->centroid_global, matchedCentroids[*closestMatchingInd]));

      } else {
        currentUpdates_.emplace_back(projDatas[i].obs->id, projDatas[i].centroid_cam.x);
      }
    }
  }
  std::cout << '[' << which << "]: Matchings Size: " << matchings_num << std::endl;
}

void Matcher::autocalib(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs, const std::vector<Matching> &matchings) {
  // Fill the request parameters
  ccat::CalibReq req;
  req.request.euler_angles.resize(3);
  Eigen::Vector3d::Map(&req.request.euler_angles[0], 3) = extrinsics_.rotation().eulerAngles(0, 1, 2);
  req.request.translation.resize(3);
  Eigen::Vector3d::Map(&req.request.translation[0], 3) = Eigen::Vector3d(extrinsics_.translation().col(0).head(3));
  req.request.camera_matrix = std::vector<float>(intrinsics_.data(), intrinsics_.data() + intrinsics_.size());
  req.request.bbsCentroids.reserve(matchings.size());
  req.request.obsCentroids.reserve(matchings.size());
  for (size_t projInd = 0; projInd < matchings.size(); projInd++) {
    if (bool(matchings[projInd])) {
      req.request.obsCentroids.push_back(projections[projInd].data->obs->temp.centroid_local.gmPoint());
      req.request.bbsCentroids.push_back(bbCentroidAndHeight(bbs.poses[matchings[projInd].bbInd()]).gmPoint());
    }
  }
  req.request.bbsCentroids.shrink_to_fit();
  req.request.obsCentroids.shrink_to_fit();

  // Call the calibration service
  if (calibSrv_.call(req)) {
    // Update the extrinsics
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

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

Matcher::Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, const Which &which) : which(which), params_(params), intrinsics_(params.intrinsics.data()), vis_(params) {
  hasValidData_ = false;
  calibrated_ = false;

  // Extrinsics
  Eigen::Quaterniond rotationQ(
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.extrinsics.euler_angles[2], Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(params_.extrinsics.translation.data());

  // Calibration service instantiation
  calibSrv_ = nh->serviceClient<ccat::CalibReq>(params_.service_addr);
}

void Matcher::cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level) {
  ROS_WARN("cfg callback");
  Eigen::Quaterniond rotationQ(Eigen::AngleAxisd(config.roll, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(config.pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(config.yaw, Eigen::Vector3d::UnitZ()));
  extrinsics_.linear() = rotationQ.toRotationMatrix();
  extrinsics_.translation() = Eigen::Vector3d(config.x, config.y, config.z);
}

void Matcher::run(const std::vector<Observation::Ptr> &observations, const geometry_msgs::PoseArray::ConstPtr &bbs) {
  currentUpdates_.clear();
  if (bbs == nullptr) {
    ROS_WARN("Intent to match with invalid BBs");
    return;
  }
  if (bbs->poses.size() == 0) return;

  if (calibrated_) hasValidData_ = true;

  // 1: Transform the observations (pcls AND centroid) to camera space.
  std::vector<ProjectionData> projDatas(observations.size());
  for (size_t i = 0; i < observations.size(); i++) {
    projDatas[i].obsInd = i;
    projDatas[i].obs = observations[i];
    projDatas[i].centroid_cam = observations[i]->temp.centroid_local.transformed(extrinsics_);
    pcl::transformPointCloud(*observations[i]->temp.pcl_local, *projDatas[i].pcl_cam, extrinsics_);
  }

  // 2: Filter the points so only the points in front of the camera (visibles)
  // are left. Here we are removing the points that have (z < 0.0).
  pcl::ConditionalRemoval<PCLPoint> xFilter;
  pcl::ConditionAnd<PCLPoint>::Ptr xFilterCond(pcl::make_shared<pcl::ConditionAnd<PCLPoint>>());
  xFilterCond->addComparison(pcl::make_shared<pcl::FieldComparison<PCLPoint>>("x", pcl::ComparisonOps::GE, 0.0));
  xFilter.setCondition(xFilterCond);
  for (const ProjectionData &projData : projDatas) {
    xFilter.setInputCloud(projData.pcl_cam);
    xFilter.filter(*projData.pcl_cam);
  }

  // If debug, publish all the cones in camera coordinates
  if (params_.debug) {
    std::vector<PCL::Ptr> pcls(projDatas.size());
    std::transform(projDatas.begin(), projDatas.end(), pcls.begin(),
                   [](const ProjectionData &pd) -> PCL::Ptr { return pd.pcl_cam; });
    vis_.publishPCLs(pcls);
  }

  // 3: Create the projections of the points, that is:
  // - Transform all the points to image space
  // - Save only the observations which have at least one point on the image
  std::vector<Projection> projs;
  projections(projDatas, projs);

  // 4: Now obtain the matches
  std::vector<Matching> matchings;
  computeMatchings(projs, *bbs, matchings);

  // If debug:
  // - publish the images showing the projections (aka calibration)
  // - publish markers showing the actual matchings
  if (params_.debug) {
    std::vector<std::pair<std::vector<cv::Point2d>, cv::Point2d>> projsToPaint(projs.size());
    std::transform(projs.begin(), projs.end(), projsToPaint.begin(),
                   [](const Projection &p) -> std::pair<std::vector<cv::Point2d>, cv::Point2d> { return {p.projPoints, p.imageCentroid}; });
    vis_.publishProjectedImg(projsToPaint, *bbs, matchings);

    std::vector<geometry_msgs::Point> projsCentroids(projs.size());
    std::transform(projs.begin(), projs.end(), projsCentroids.begin(),
                   [](const Projection &p) -> geometry_msgs::Point { return p.data->obs->temp.centroid_local.gmPoint(); });
    vis_.publishMatchingMarkers(projsCentroids, *bbs, matchings);
  }

  // 5: Update the cones data
  updateData(projs, projDatas, *bbs, matchings);

  // 6: Autocalibrate
  autocalib(projs, *bbs, matchings);
}

const std::vector<ConeUpdate> &Matcher::getData() const { return currentUpdates_; }

const bool &Matcher::hasValidData() const { return hasValidData_; }