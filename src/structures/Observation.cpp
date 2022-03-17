#include "structures/Observation.hpp"

/**
 * CONSTRUCTORS
 */
Observation::Observation() : pcl(pcl::make_shared<PCL>()), confidence(0.0) {}
Observation::Observation(const PCL::Ptr &pcl, const float &confidence, const size_t &id) : pcl(pcl), confidence(confidence), id(id) {
  centroid = computeCentroid(*this->pcl);
}

Observation::Observation(const as_msgs::Observation &obs, const size_t &id) : pcl(pcl::make_shared<PCL>()), centroid(Point(obs.centroid)), confidence(obs.confidence), id(id) {
  pcl::fromROSMsg(obs.cloud, *pcl);
  // invert y and z axis
  for (auto &p : pcl->points) {
    //p.y *= -1;
    p.z *= -1;
  }
}

Observation::Observation(const std::list<Observation::Ptr> &observationsToMean) : pcl(pcl::make_shared<PCL>()), confidence(0.0) {
  for (auto it = observationsToMean.begin(); it != observationsToMean.end(); it++) {
    if (it == observationsToMean.begin()) id = (*it)->id;
    *pcl += *((*it)->pcl);
    confidence += (*it)->confidence;
  }
  centroid = computeCentroid(*pcl);
  confidence /= observationsToMean.size();
}

/**
 * DESTRUCTORS
 */
Observation::~Observation() {}

/**
 * PROTECTED METHODS
 */
Point Observation::computeCentroid(const PCL &pcl) {
  PCLPoint p;
  pcl::computeCentroid<PCLPoint, PCLPoint>(pcl, p);
  return Point(p);
}
