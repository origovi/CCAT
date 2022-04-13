#include "modules/Tracker.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */

Tracker::Tracker() {
  lastId_ = 0;
}

/**
 * PRIVATE METHODS
 */

void Tracker::updateCurrentCones(const Eigen::Affine3d &carTf) {
  currentCones_.header.stamp = ros::Time::now();
  currentCones_.cones.resize(trackings_.size());
  size_t trackInd = 0;
  for (std::list<Tracking>::const_iterator it = trackings_.begin(); it != trackings_.end(); it++) {
    currentCones_.cones[trackInd] = it->getASCone(carTf);
    trackInd++;
  }
}

void Tracker::publishMarkers() const {
  visualization_msgs::MarkerArray maGlobal, maBaseLink;
  maGlobal.markers.resize(2*currentCones_.cones.size());
  maBaseLink.markers.resize(2*currentCones_.cones.size());
  visualization_msgs::Marker mCommon, mCommonT;
  mCommon.lifetime = ros::Duration();
  mCommon.header.stamp = currentCones_.header.stamp;
  mCommon.action = visualization_msgs::Marker::ADD;
  if (params_.fancy_markers) {
    mCommon.type = visualization_msgs::Marker::MESH_RESOURCE;
    mCommon.scale.x = 2.0;
    mCommon.scale.y = 2.0;
    mCommon.scale.z = 2.0;
  } else {
    mCommon.type = visualization_msgs::Marker::CYLINDER;
    mCommon.scale.x = 0.5;
    mCommon.scale.y = 0.5;
    mCommon.scale.z = 1.0;
  }
  mCommon.pose.orientation.w = 1.0;
  mCommon.color.a = 1.0;
  mCommonT = mCommon;
  mCommonT.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  for (size_t i = 0; i < currentCones_.cones.size(); i++) {
    mCommon.id = currentCones_.cones[i].id;
    mCommonT.id = lastId_+1+i;
    mCommonT.text = std::to_string(currentCones_.cones[i].id);
    switch (currentCones_.cones[i].type) {
      case 0:  // Yellow
        mCommon.color.r = 1.0f;
        mCommon.color.g = 1.0f;
        mCommon.color.b = 0.0f;
        if (params_.fancy_markers) mCommon.mesh_resource = "package://ccat/config/meshes/cone_yellow.dae";
        break;

      case 1:  // Blue
        mCommon.color.r = 0.0f;
        mCommon.color.g = 0.0f;
        mCommon.color.b = 1.0f;
        if (params_.fancy_markers) mCommon.mesh_resource = "package://ccat/config/meshes/cone_blue.dae";
        break;

      case 2:  // Small Orange
        mCommon.color.r = 1.0f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.0f;
        if (params_.fancy_markers) mCommon.mesh_resource = "package://ccat/config/meshes/cone_orange.dae";
        break;

      case 3:  // Big Orange
        mCommon.color.r = 1.0f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.0f;
        if (params_.fancy_markers) mCommon.mesh_resource = "package://ccat/config/meshes/cone_orange_big.dae";
        break;

      default:  // Unclassified
        mCommon.color.r = 0.5f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.5f;
        if (params_.fancy_markers) mCommon.mesh_resource = "package://ccat/config/meshes/cone_gray.dae";
        break;
    }
    visualization_msgs::Marker &mG = maGlobal.markers[i];
    visualization_msgs::Marker &mGT = maGlobal.markers[currentCones_.cones.size()+i];
    visualization_msgs::Marker &mB = maBaseLink.markers[i];
    visualization_msgs::Marker &mBT = maBaseLink.markers[currentCones_.cones.size()+i];
    mG = mCommon;
    mGT = mCommonT;
    mB = mCommon;
    mBT = mCommonT;
    mG.pose.position = currentCones_.cones[i].position_global;
    mG.pose.position.z = 0.325;
    mG.header.frame_id = "global";
    mGT.pose.position = mG.pose.position;
    mGT.pose.position.z = 1.0;
    mGT.header.frame_id = mG.header.frame_id;

    mB.pose.position = currentCones_.cones[i].position_base_link;
    mB.pose.position.z = 0.325;
    mB.header.frame_id = "base_link";
    mBT.pose.position = mB.pose.position;
    mBT.pose.position.z = 1.0;
    mBT.header.frame_id = mB.header.frame_id;
  }
  markerBaseLinkPub_.publish(maBaseLink);
  markerGlobalPub_.publish(maGlobal);
}

void Tracker::getTrackingPoints(std::vector<Point> &points, std::vector<Tracking *> &trackingPtrs) {
  points.resize(trackings_.size());
  trackingPtrs.resize(trackings_.size());
  size_t ind = 0;
  for (std::list<Tracking>::iterator it = trackings_.begin(); it != trackings_.end(); it++) {
    points[ind] = it->position();
    trackingPtrs[ind] = &(*it);
    ind++;
  }
}

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */

Tracker &Tracker::getInstance() {
  static Tracker matcher;
  return matcher;
}

/* Init */

void Tracker::init(ros::NodeHandle *const &nh, const Params::Tracker &params) {
  nh_ = nh;
  params_ = params;
  if (params_.debug) {
    markerBaseLinkPub_ = nh->advertise<visualization_msgs::MarkerArray>(params_.topics.output.markersBaseLink, 1);
    markerGlobalPub_ = nh->advertise<visualization_msgs::MarkerArray>(params_.topics.output.markersGlobal, 1);
  }
}

void Tracker::run(const std::vector<Cone> &cones, const Eigen::Affine3d &carTf) {
  if (trackings_.empty()) {
    for (const Cone &cone : cones) {
      trackings_.emplace_back(cone, lastId_++);
    }
  } else {
    std::vector<Point> points(trackings_.size());
    std::vector<Tracking *> trackingPtrs(trackings_.size());
    getTrackingPoints(points, trackingPtrs);
    KDTree tucutu(points);
    for (size_t i = 0; i < cones.size(); i++) {
      const Point &pointToSearch = cones[i].observation->centroid_global;
      size_t nearestInd = *tucutu.nearest_index(pointToSearch);
      double distSqToOldPos = Point::distSq(points[nearestInd], pointToSearch);
      // We have observed a known cone
      if (distSqToOldPos <= params_.same_cone_max_distSq) {
        trackingPtrs[nearestInd]->addCone(cones[i], distSqToOldPos);
      }
      // We have observed a new cone
      else {
        trackings_.emplace_back(cones[i], lastId_++);
      }
    }
  }
  updateCurrentCones(carTf);
}

// void Tracker::accumulate(const std::vector<Observation> &obs) {

// }

/* Callbacks */

/* Getters */

const as_msgs::ConeArray &Tracker::getData() const {
  // Publish markers, if debug
  if (params_.debug) {
    publishMarkers();
  }
  return currentCones_;
}

bool Tracker::hasData() const {
  return !currentCones_.cones.empty();
}