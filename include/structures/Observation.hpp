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
 protected:
  static Point computeCentroid(const PCL &pcl);

 public:
  using Ptr = std::shared_ptr<Observation>;
  using ConstPtr = std::shared_ptr<const Observation>;
  
  /**
   * PUBLIC CONSTRUCTORS AND DESTRUCTOR
   */

  Observation();
  Observation(const PCL &pcl, const float &confidence, const size_t &id);
  Observation(const PCL &pcl, const Point &centroid, const float &confidence, const size_t &id);
  Observation(const as_msgs::Observation &obs, const size_t &id);
  Observation(const std::list<const Observation*> &observationsToMean);

  /* PUBLIC ATTRIBUTES */

  PCL pcl;

  Point centroid_global;

  /**
   * @brief The data stored inside this struct will be invalid
   * among iterations. That's why it's called \a temp.
   */
  struct {
    Point centroid_local;
    Point centroid_camera;
    PCL::Ptr pcl = pcl::make_shared<PCL>();
    double distToCar;
  } temp;


  double confidence;
  size_t id;

  /* PUBLIC METHODS */

  void updateLocal(const Eigen::Affine3d &carTf);
};

#endif  // STRUCTURES_OBSERVATION_HPP