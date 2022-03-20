#ifndef CONVERSIONS_HH
#define CONVERSIONS_HH

#include <as_msgs/ObservationArray.h>

#include <cmath>
#include <vector>
#include "structures/Observation.hpp"

namespace cvrs {

void as_obsVec2ObsVec(const std::vector<as_msgs::Observation>& observations, std::vector<Observation::Ptr> &res);
void obsVec2PointVec(const std::vector<Observation::Ptr>& observations, std::vector<Point> &res);
void obs2PCLs(const std::vector<Observation::Ptr> &observations, std::vector<PCL::Ptr> &res);

}  // namespace cvrs

#endif  // CONVERSIONS_HPP