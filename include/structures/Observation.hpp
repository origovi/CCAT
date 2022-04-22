#ifndef STRUCTURES_OBSERVATION_HPP
#define STRUCTURES_OBSERVATION_HPP

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

#include <cmath>

#include "as_msgs/Observation.h"
#include "structures/Point.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;

class Observation {
 public:
  using Ptr = std::shared_ptr<Observation>;
  using ConstPtr = std::shared_ptr<const Observation>;
  
  /**
   * PUBLIC CONSTRUCTORS AND DESTRUCTOR
   */

  Observation();
  Observation(const PCL &pcl_global, const float &confidence);
  Observation(const PCL &pcl_global, const Point &centroid, const float &confidence);
  Observation(const as_msgs::Observation &obs);
  Observation(const std::list<const Observation*> &observationsToMean);

  /* PUBLIC ATTRIBUTES */

  PCL pcl_global;

  Point centroid_global;

  /**
   * @brief The data stored inside this struct will be invalid
   * among iterations. That's why it's called \a temp.
   */
  struct {
    Point centroid_local;
    PCL::Ptr pcl_local = pcl::make_shared<PCL>();
    double distToCar;
  } temp;


  double confidence;
  size_t id;

  /* PUBLIC METHODS */
  static Point computeCentroid(const PCL &pcl);
  void updateLocal(const Eigen::Affine3d &carTf);
};

#endif  // STRUCTURES_OBSERVATION_HPP