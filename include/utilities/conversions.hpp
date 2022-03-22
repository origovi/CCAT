#ifndef CONVERSIONS_HH
#define CONVERSIONS_HH

#include <as_msgs/ObservationArray.h>
#include <as_msgs/ConeArray.h>
#include <as_msgs/Cone.h>

#include <cmath>
#include <vector>
#include "structures/Observation.hpp"
#include "structures/Cone.hpp"

namespace cvrs {

void as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation> &res);
void obsVec2PointVec(const std::vector<Observation>& observations, std::vector<Point> &res);
void obs2ObsPtr(const std::vector<Observation> &observations, std::vector<Observation::Ptr> &res);
void obs2PCLs(const std::vector<Observation::Ptr> &observations, std::vector<PCL::Ptr> &res);
void coneVec2As_ConeArray(const std::vector<Cone> &cones, as_msgs::ConeArray &res);
}  // namespace cvrs

#endif  // CONVERSIONS_HPP