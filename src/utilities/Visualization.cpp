#include "utilities/Visualization.hpp"

Visualization::Visualization(ros::NodeHandle *const nh) : nh_(nh) {}

ros::Publisher &Visualization::pubPcl(const std::string &topic, const size_t &defQueueSize) {
  return pubPcl_.get(*nh_, topic, defQueueSize);
}
ros::Publisher &Visualization::pubMA(const std::string &topic, const size_t &defQueueSize) {
  return pubMA_.get(*nh_, topic, defQueueSize);
}
ros::Publisher &Visualization::pubM(const std::string &topic, const size_t &defQueueSize) {
  return pubM_.get(*nh_, topic, defQueueSize);
}
image_transport::Publisher &Visualization::pubImg(const std::string &topic, const size_t &defQueueSize) {
  return pubImg_.get(*nh_, topic, defQueueSize);
}