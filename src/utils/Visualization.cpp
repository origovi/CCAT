#include "utils/Visualization.hpp"

// Static attributes declaration
ros::NodeHandle *Visualization::nh_ = nullptr;
Visualization::DB<Visualization::RosPub<sensor_msgs::PointCloud2>> Visualization::pubPcl_;
Visualization::DB<Visualization::RosPub<visualization_msgs::MarkerArray>> Visualization::pubMA_;
Visualization::DB<Visualization::RosPub<visualization_msgs::Marker>> Visualization::pubM_;
Visualization::DB<Visualization::ImgPub> Visualization::pubImg_;

/* -------------------------------- Protected ------------------------------- */

Visualization::Visualization() {}

ros::Publisher &Visualization::pubPcl(const std::string &topic, const size_t &defQueueSize) {
  return pubPcl_.get(topic, defQueueSize);
}
ros::Publisher &Visualization::pubMA(const std::string &topic, const size_t &defQueueSize) {
  return pubMA_.get(topic, defQueueSize);
}
ros::Publisher &Visualization::pubM(const std::string &topic, const size_t &defQueueSize) {
  return pubM_.get(topic, defQueueSize);
}
image_transport::Publisher &Visualization::pubImg(const std::string &topic, const size_t &defQueueSize) {
  return pubImg_.get(topic, defQueueSize);
}

/* --------------------------------- Public --------------------------------- */

void Visualization::init(ros::NodeHandle *const nh) {
  nh_ = nh;
}