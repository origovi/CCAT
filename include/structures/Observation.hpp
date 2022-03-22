#ifndef OBSERVATION_HPP
#define OBSERVATION_HPP

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

#include "as_msgs/Observation.h"
#include "structures/Point.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;

class Observation {
 protected:
  static Point computeCentroid(const PCL &pcl);

 public:
  using Ptr = std::shared_ptr<Observation>;
  using ConstPtr = std::shared_ptr<const Observation>;
  
  /**
   * PUBLIC CONSTRUCTORS AND DESTRUCTOR
   */

  Observation();
  Observation(const PCL::Ptr &pcl, const float &confidence, const size_t &id);
  Observation(const PCL::Ptr &pcl, const Point &centroid, const float &confidence, const size_t &id);
  Observation(const as_msgs::Observation &obs, const size_t &id);
  Observation(const std::list<const Observation*> &observationsToMean);
  ~Observation();

  /* PUBLIC ATTRIBUTES */

  PCL::Ptr pcl;

  Point centroid_global;
  Point centroid_base_link;
  Point centroid_transformed;

  double distToCar;

  double confidence;
  size_t id;
};

#endif  // OBSERVATION_HPP