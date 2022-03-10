#ifndef CONVERSIONS_HH
#define CONVERSIONS_HH

#include <as_msgs/ObservationArray.h>

#include <cmath>
#include <vector>
#include "Observation.hpp"

namespace cvrs {

std::vector<Observation> as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations) {
  std::vector<Observation> res(observations.size());
  for (int i = 0; i < observations.size(); ++i) {
    res[i] = Observation(observations[i]);
  }
  return res;
}

std::vector<Point> obsVec2PointVec(const std::vector<Observation>& observations) {
  std::vector<Point> res(observations.size());
  for (int i = 0; i < observations.size(); ++i) {
    res[i] = observations[i].centroid;
  }
  return res;
}

}  // namespace cvrs

#endif  // CONVERSIONS_HPP