#include "Matcher.hpp"

/**
 * CONSTRUCTORS
 */
Matcher::Matcher(const Params::Matcher &params) : params_(params), hasData_(false) {}

/**
 * DESTRUCTORS
 */
Matcher::~Matcher() {}

/**
 * PRIVATE METHODS
 */
std::vector<PCL::Ptr> Matcher::reconstructions(const std::vector<Observation> &observations) const {
  std::vector<PCL::Ptr> res(observations.size());
  pcl::CropBox<PCLPoint> cropBoxFilter;
  cropBoxFilter.setInputCloud(actualMap_);
  for (size_t i = 0; i < res.size(); ++i) {
    Eigen::Vector4f min_pt(-1.0f, -1.0f, -1.0f, 1.0f);
    Eigen::Vector4f max_pt(1.0f, 1.0f, 1.0f, 1.0f);
    cropBoxFilter.setMin(Eigen::Vector4f(observations[i].p.x - params_.reconstruct.cone_size_width * params_.reconstruct.safety_factor / 2, observations[i].p.y, observations[i].p.z, 1.0f));
    cropBoxFilter.setMax(Eigen::Vector4f(observations[i].p.x, observations[i].p.y, observations[i].p.z, 1.0f));
    cropBoxFilter.filter(*res[i]);
  }
  return res;
}

/**
 * PUBLIC METHODS
 */
/* Singleton pattern */
Matcher &Matcher::getInstance(const Params::Matcher &params) {
  static Matcher matcher(params);
  return matcher;
}

/* Callbacks */
void Matcher::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  if (cloud_msg->data.empty())
    ROS_WARN("Trying to read empty pointcloud");
  else
    pcl::fromROSMsg(*cloud_msg, *actualMap_);
}

void Matcher::locationAndBbsCbk(const nav_msgs::Odometry::ConstPtr &carPos, const geometry_msgs::PoseArray::ConstPtr &leftDetections, const geometry_msgs::PoseArray::ConstPtr &rightDetections) {
  carLocation_ = carPos;
  leftDetections_ = leftDetections;
  rightDetections_ = rightDetections;
  hasData_ = true;
}

/* Functions */
void Matcher::run(const std::vector<Observation> &observations) {
}

/* Getters */
const bool &Matcher::hasData() const {
  return hasData_;
}
