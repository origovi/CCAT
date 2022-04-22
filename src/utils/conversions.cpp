#include "utils/conversions.hpp"

void cvrs::as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation> &res) {
  res.clear();
  res.reserve(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res.emplace_back(observations[i]);
  }
}

void cvrs::obsVec2PointVec(const std::vector<Observation>& observations, std::vector<Point> &res) {
  res.resize(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    res[i] = observations[i].centroid_global;
  }
}