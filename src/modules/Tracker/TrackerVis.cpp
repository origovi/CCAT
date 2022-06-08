#include "modules/Tracker/TrackerVis.hpp"

/* --------------------------------- Private -------------------------------- */

void TrackerVis::paintMarkerFromType(visualization_msgs::Marker &m, const uint8_t &type) {
  if (params_.fancy_markers) {
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.scale.x = 2.0;
    m.scale.y = 2.0;
    m.scale.z = 2.0;
  } else {
    m.type = visualization_msgs::Marker::CYLINDER;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 1.0;
  }

  m.color.a = 1.0f;
  switch (type) {
    case 0:  // Yellow
      m.color.r = 1.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      if (params_.fancy_markers) m.mesh_resource = "package://ccat/config/meshes/cone_yellow.dae";
      break;

    case 1:  // Blue
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
      if (params_.fancy_markers) m.mesh_resource = "package://ccat/config/meshes/cone_blue.dae";
      break;

    case 2:  // Small Orange
      m.color.r = 1.0f;
      m.color.g = 0.5f;
      m.color.b = 0.0f;
      if (params_.fancy_markers) m.mesh_resource = "package://ccat/config/meshes/cone_orange.dae";
      break;

    case 3:  // Big Orange
      m.color.r = 1.0f;
      m.color.g = 0.5f;
      m.color.b = 0.0f;
      if (params_.fancy_markers) m.mesh_resource = "package://ccat/config/meshes/cone_orange_big.dae";
      break;

    default:  // Unclassified
      m.color.r = 0.5f;
      m.color.g = 0.5f;
      m.color.b = 0.5f;
      if (params_.fancy_markers) m.mesh_resource = "package://ccat/config/meshes/cone_gray.dae";
      break;
  }
}

/* --------------------------------- Public --------------------------------- */

void TrackerVis::init(const Params::Tracker &params) {
  params_ = params;
}

void TrackerVis::publishMergedMarkers(const std::vector<ConeUpdate> &coneUpdates, const std::map<size_t, Cone> &cones) {
  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(1 + coneUpdates.size());
  visualization_msgs::Marker m;
  //m.lifetime = ros::Duration();
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "base_link";
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 1.0;
  size_t id = 0;

  // Delete last iteration's markers
  m.id = id++;
  m.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(m);
  m.action = visualization_msgs::Marker::ADD;

  for (const ConeUpdate &cU : coneUpdates) {
    // If it has a match
    if (bool(cU)) {
      std::map<size_t, Cone>::const_iterator it = cones.find(cU.id);
      if (it != cones.end()) {
        m.pose.position = it->second.obs->temp.centroid_local.gmPoint();
        if (params_.markers_on_ground) m.pose.position.z = 0.325;
        m.id = id++;
        paintMarkerFromType(m, static_cast<uint8_t>(cU.type));
        ma.markers.push_back(m);
      }
    }
  }
  pubMA(params_.topics.output.mergedMarkers).publish(ma);
}

void TrackerVis::publishFinalMarkers(const as_msgs::ConeArray &currentCones) {
  visualization_msgs::MarkerArray maGlobal, maBaseLink;
  maGlobal.markers.resize(1 + (params_.show_markers_id ? 2 : 1) * currentCones.cones.size());
  maBaseLink.markers.resize(1 + (params_.show_markers_id ? 2 : 1) * currentCones.cones.size());
  visualization_msgs::Marker mCommon, mCommonT;
  mCommon.lifetime = ros::Duration();
  mCommon.header.stamp = currentCones.stamp;
  mCommon.action = visualization_msgs::Marker::ADD;
  mCommon.pose.orientation.w = 1.0;
  mCommon.color.a = 1.0;
  mCommon.scale.z = 1.0;
  mCommonT = mCommon;
  mCommonT.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // Find biggest ID, the ID markers will have an id > biggestId
  size_t biggestId = 0;
  for (size_t i = 0; i < currentCones.cones.size(); i++) {
    biggestId = (currentCones.cones[i].id > biggestId) ? currentCones.cones[i].id : biggestId;
  }
  maGlobal.markers[0] = mCommon;
  maGlobal.markers[0].id = -1;
  maGlobal.markers[0].header.frame_id = "global";
  maGlobal.markers[0].action = visualization_msgs::Marker::DELETEALL;
  maBaseLink.markers[0] = mCommon;
  maBaseLink.markers[0].id = -1;
  maBaseLink.markers[0].header.frame_id = "base_link";
  maBaseLink.markers[0].action = visualization_msgs::Marker::DELETEALL;
  for (size_t i = 0; i < currentCones.cones.size(); i++) {
    mCommon.id = currentCones.cones[i].id;
    mCommonT.id = biggestId + 1 + i;
    mCommonT.text = std::to_string(currentCones.cones[i].id);
    paintMarkerFromType(mCommon, currentCones.cones[i].type);
    visualization_msgs::Marker &mG = maGlobal.markers[i+1];
    visualization_msgs::Marker &mB = maBaseLink.markers[i+1];

    mG = mCommon;
    mG.pose.position = currentCones.cones[i].position_global;
    if (params_.markers_on_ground) mG.pose.position.z = 0.325;
    mG.header.frame_id = "global";

    mB = mCommon;
    mB.pose.position = currentCones.cones[i].position_baseLink;
    if (params_.markers_on_ground) mB.pose.position.z = 0.325;
    mB.header.frame_id = "base_link";

    if (params_.show_markers_id) {
      visualization_msgs::Marker &mGT = maGlobal.markers[currentCones.cones.size() + i + 1];
      visualization_msgs::Marker &mBT = maBaseLink.markers[currentCones.cones.size() + i + 1];
      mGT = mCommonT;
      mBT = mCommonT;
      mGT.pose.position = mG.pose.position;
      mGT.pose.position.z += 0.675;
      mGT.header.frame_id = mG.header.frame_id;
      mBT.pose.position = mB.pose.position;
      mBT.pose.position.z += 0.675;
      mBT.header.frame_id = mB.header.frame_id;
    }
  }
  pubMA(params_.topics.output.finalMarkersBaseLink).publish(maBaseLink);
  pubMA(params_.topics.output.finalMarkersGlobal).publish(maGlobal);
}
