#include "utilities/conversions.hpp"

void cvrs::as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = Observation(observations[i], i);
  }
}

void cvrs::obsVec2PointVec(const std::vector<Observation>& observations, std::vector<Point> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = observations[i].centroid_global;
  }
}

void cvrs::obs2ObsPtr(const std::vector<Observation> &observations, std::vector<Observation::Ptr> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = std::make_shared<Observation>(observations[i]);
    res[i]->pcl = pcl::make_shared<PCL>(*res[i]->pcl);
  }
}

void cvrs::obs2PCLs(const std::vector<Observation::Ptr> &observations, std::vector<PCL::Ptr> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); i++) {
    res[i] = pcl::make_shared<PCL>(*(observations[i]->pcl));
  }
}

void cvrs::coneVec2As_ConeArray(const std::vector<Cone> &cones, as_msgs::ConeArray &res) {
  res.cones.resize(cones.size());
  for (size_t i = 0; i < cones.size(); i++) {
    res.cones[i].id = cones[i].observation->id;
    res.cones[i].position_base_link = cones[i].observation->centroid_base_link.gmPoint();
    res.cones[i].position_global = cones[i].observation->centroid_global.gmPoint();
    res.cones[i].type = cones[i].typeToAsMsgs();
  }
}
