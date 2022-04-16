#ifndef UTILS_CONVERSIONS_HPP
#define UTILS_CONVERSIONS_HPP

#include <as_msgs/ObservationArray.h>
#include <as_msgs/ConeArray.h>
#include <as_msgs/Cone.h>

#include <cmath>
#include <vector>
#include "structures/Observation.hpp"

namespace cvrs {

void as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation> &res);
void obsVec2PointVec(const std::vector<Observation>& observations, std::vector<Point> &res);

}  // namespace cvrs

#endif  // UTILS_CONVERSIONS_HPP