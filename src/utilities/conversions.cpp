#include "utilities/conversions.hpp"

void cvrs::as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation::Ptr> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = std::make_shared<Observation>(Observation(observations[i], i));
  }
}

void cvrs::obsVec2PointVec(const std::vector<Observation::Ptr>& observations, std::vector<Point> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = observations[i]->centroid;
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
  as_msgs::Cone cone;
  for (size_t i = 0; i < cones.size(); i++) {
    cone.id = cones[i].observation->id;
    cone.position_base_link = cones[i].observation->centroid.gmPoint();
    cone.type = cones[i].typeToAsMsgs();
    res.cones[i] = cone;
  }
}
