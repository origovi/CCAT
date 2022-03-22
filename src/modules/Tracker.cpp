#include "modules/Tracker.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */

Tracker::Tracker() {}
Tracker::~Tracker() {}

/**
 * PRIVATE METHODS
 */

void Tracker::publishMarkers() const {
  visualization_msgs::MarkerArray maGlobal, maBaseLink;
  maGlobal.markers.resize(currentCones_.cones.size());
  maBaseLink.markers.resize(currentCones_.cones.size());
  for (size_t i = 0; i < currentCones_.cones.size(); i++) {
    visualization_msgs::Marker mCommon;
    mCommon.lifetime = ros::Duration(0.1);
    mCommon.header.stamp = currentCones_.header.stamp;
    mCommon.id = currentCones_.cones[i].id;
    mCommon.type = visualization_msgs::Marker::CYLINDER;
    mCommon.scale.x = 0.5;
    mCommon.scale.y = 0.5;
    mCommon.scale.z = 0.5;
    mCommon.pose.orientation.x = 0.0;
    mCommon.pose.orientation.y = 0.0;
    mCommon.pose.orientation.z = 0.0;
    mCommon.pose.orientation.w = 1.0;
    mCommon.color.a = 1.0;
    switch (currentCones_.cones[i].type) {
      case 0:  // Yellow
        mCommon.color.r = 1.0f;
        mCommon.color.g = 1.0f;
        mCommon.color.b = 0.0f;
        break;

      case 1:  // Blue
        mCommon.color.r = 0.0f;
        mCommon.color.g = 0.0f;
        mCommon.color.b = 1.0f;
        break;

      case 2:  // Small Orange
        mCommon.color.r = 1.0f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.0f;
        break;

      case 3:  // Big Orange
        mCommon.color.r = 1.0f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.0f;
        break;

      default:  // Unclassified
        mCommon.color.r = 0.5f;
        mCommon.color.g = 0.5f;
        mCommon.color.b = 0.5f;
        break;
    }
    visualization_msgs::Marker &mG = maGlobal.markers[i];
    visualization_msgs::Marker &mB = maBaseLink.markers[i];
    mG = mCommon;
    mB = mCommon;
    mG.pose.position = currentCones_.cones[i].position_global;
    mB.pose.position = currentCones_.cones[i].position_base_link;
    mG.header.frame_id = "global";
    mB.header.frame_id = "base_link";
  }
  markerBaseLinkPub_.publish(maBaseLink);
  markerGlobalPub_.publish(maGlobal);
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

void Tracker::run(const std::vector<Cone> &cones) {
  // Convert the array to as_msgs
  cvrs::coneVec2As_ConeArray(cones, currentCones_);
  currentCones_.header.stamp = ros::Time::now();
}

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