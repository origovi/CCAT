#include "structures/Observation.hpp"

/**
 * CONSTRUCTORS
 */
Observation::Observation() : confidence(0.0) {}
Observation::Observation(const PCL &pcl_global, const float &confidence, const size_t &id) : pcl_global(pcl_global), confidence(confidence), id(id) {
  centroid_global = computeCentroid(this->pcl_global);
}

Observation::Observation(const as_msgs::Observation &obs, const size_t &id) : centroid_global(Point(obs.centroid)), confidence(obs.confidence), id(id) {
  pcl::fromROSMsg(obs.cloud, pcl_global);
  // invert y and z axis
  for (auto &p : pcl_global.points) {
    p.y *= -1;
    // p.z *= -1;
  }
}

Observation::Observation(const std::list<const Observation*> &observationsToMean) : confidence(0.0) {
  for (auto it = observationsToMean.begin(); it != observationsToMean.end(); it++) {
    if (it == observationsToMean.begin()) id = (*it)->id;
    pcl_global += (*it)->pcl_global;
    confidence += (*it)->confidence;
  }
  centroid_global = computeCentroid(pcl_global);
  confidence /= observationsToMean.size();
}

Point Observation::computeCentroid(const PCL &pcl) {
  PCLPoint p;
  pcl::computeCentroid<PCLPoint, PCLPoint>(pcl, p);
  return Point(p);
}

void Observation::updateLocal(const Eigen::Affine3d &carTf) {
  temp.centroid_local = centroid_global.transformed(carTf);
  pcl::transformPointCloud(pcl_global, *temp.pcl_local, carTf);
}
