#ifndef CONVERSIONS_HH
#define CONVERSIONS_HH

#include <as_msgs/ObservationArray.h>

#include <cmath>

#include "KDTree.hpp"

namespace cvrs {

pointVec observationsToPointVec(const std::vector<as_msgs::Observation>& observations) {
  pointVec res(observations.size());
  for (int i = 0; i < observations.size(); ++i) {
    res[i] = Point(observations[i]);
  }
  return res;
}

}  // namespace cvrs

#endif  // CONVERSIONS_HPP