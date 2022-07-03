#include "modules/Preproc/PreprocVis.hpp"

/* --------------------------------- Public --------------------------------- */

void PreprocVis::init(const Params::Preproc &params) {
  params_ = params;
}

void PreprocVis::publishInputMarkers(const std::vector<Observation> &observations) {
  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(1 + observations.size());
  visualization_msgs::Marker m;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "global";
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.type = visualization_msgs::Marker::SPHERE;
  m.color.a = 1.0;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  size_t id = 0;

  // Delete last iteration's markers
  m.id = id++;
  m.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(m);
  m.action = visualization_msgs::Marker::ADD;

  for (const Observation &obs : observations) {
    m.pose.position = obs.centroid_global.gmPoint();
    m.id = id++;
    ma.markers.push_back(m);
  }
  pubMA(params_.topics.output.input_markers).publish(ma);
}
