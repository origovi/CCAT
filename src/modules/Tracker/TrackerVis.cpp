#include "modules/Tracker/TrackerVis.hpp"

/* --------------------------------- Private -------------------------------- */

/* --------------------------------- Public --------------------------------- */

void TrackerVis::init(const Params::Tracker &params) {
  params_ = params;
}

void TrackerVis::publishMarkers(const as_msgs::ConeArray &currentCones) {
  visualization_msgs::MarkerArray maGlobal, maBaseLink;
  maGlobal.markers.resize(2 * currentCones.cones.size());
  maBaseLink.markers.resize(2 * currentCones.cones.size());
  visualization_msgs::Marker mCommon, mCommonT;
  mCommon.lifetime = ros::Duration();
  mCommon.header.stamp = currentCones.stamp;
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

  // Find biggest ID, the ID markers will have an id > biggestId
  size_t biggestId = 0;
  for (size_t i = 0; i < currentCones.cones.size(); i++) {
    biggestId = (currentCones.cones[i].id > biggestId) ? currentCones.cones[i].id : biggestId;
  }

  for (size_t i = 0; i < currentCones.cones.size(); i++) {
    mCommon.id = currentCones.cones[i].id;
    mCommonT.id = biggestId + 1 + i;
    mCommonT.text = std::to_string(currentCones.cones[i].id);
    switch (currentCones.cones[i].type) {
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
    visualization_msgs::Marker &mGT = maGlobal.markers[currentCones.cones.size() + i];
    visualization_msgs::Marker &mB = maBaseLink.markers[i];
    visualization_msgs::Marker &mBT = maBaseLink.markers[currentCones.cones.size() + i];
    mG = mCommon;
    mGT = mCommonT;
    mB = mCommon;
    mBT = mCommonT;
    mG.pose.position = currentCones.cones[i].position_global;
    mG.pose.position.z = 0.325;
    mG.header.frame_id = "global";
    mGT.pose.position = mG.pose.position;
    mGT.pose.position.z = 1.0;
    mGT.header.frame_id = mG.header.frame_id;

    mB.pose.position = currentCones.cones[i].position_baseLink;
    mB.pose.position.z = 0.325;
    mB.header.frame_id = "base_link";
    mBT.pose.position = mB.pose.position;
    mBT.pose.position.z = 1.0;
    mBT.header.frame_id = mB.header.frame_id;
  }
  pubMA(params_.topics.output.markersBaseLink).publish(maBaseLink);
  pubMA(params_.topics.output.markersGlobal).publish(maGlobal);
}
