#include "Observation.hpp"

/**
 * CONSTRUCTORS
 */
Observation::Observation() : pcl(pcl::make_shared<PCL>()), confidence(0.0) {}
Observation::Observation(const PCL::Ptr &pcl, const float &confidence) : pcl(pcl), confidence(confidence) {
  centroid = computeCentroid(*this->pcl);
}

Observation::Observation(const as_msgs::Observation &obs) : pcl(pcl::make_shared<PCL>()), centroid(Point(obs.centroid)), confidence(obs.confidence) {
  pcl::fromROSMsg(obs.cloud, *pcl);
}

Observation::Observation(const std::list<const Observation *> &observationsToMean) : pcl(pcl::make_shared<PCL>()), confidence(0.0) {
  for (const Observation *obs : observationsToMean) {
    *pcl += *(obs->pcl);
    confidence += obs->confidence;
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
