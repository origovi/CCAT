#ifndef OBSERVATION_HPP
#define OBSERVATION_HPP

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

#include "Point.hpp"
#include "as_msgs/Observation.h"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;

class Observation {
 protected:
  static Point computeCentroid(const PCL &pcl);

 public:
  Observation();
  Observation(const PCL::Ptr &pcl, const float &confidence);
  Observation(const PCL::Ptr &pcl, const Point &centroid, const float &confidence);
  Observation(const as_msgs::Observation &obs);
  Observation(const std::list<const Observation *> &observationsToMean);

  ~Observation();


  /* PUBLIC ATTRIBUTES */

  PCL::Ptr pcl;
  Point centroid;
  double confidence;
};

#endif  // OBSERVATION_HPP