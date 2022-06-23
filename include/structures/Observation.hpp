/**
 * @file Observation.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Observation class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

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

/**
 * @brief Represents a detection in the global 3D map and contains all
 * information regarding the cone's physical state.
 */
class Observation {
 public:
  using Ptr = std::shared_ptr<Observation>;
  using ConstPtr = std::shared_ptr<const Observation>;
  
  /* -------------------------- Public Constructors ------------------------- */

  Observation();
  Observation(const PCL &pcl_global, const float &confidence);
  Observation(const PCL &pcl_global, const Point &centroid, const float &confidence);
  Observation(const as_msgs::Observation &obs);
  Observation(const std::list<const Observation*> &observationsToMean);

  /* --------------------------- Public Attributes -------------------------- */

  /**
   * @brief The cone's point cloud in global coords.
   */
  PCL pcl_global;

  /**
   * @brief The cone's centroid in global coords.
   */
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

  /**
   * @brief The detection confidence.
   */
  double confidence;

  /**
   * @brief The unique id of this observation, same as in Cone.
   */
  size_t id;

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Computes (and updates) the centroid of the cone from a point cloud.
   * 
   * @param pcl 
   * @return Point 
   */
  static Point computeCentroid(const PCL &pcl);

  /**
   * @brief Updates the local state of the cone. All local coordinates.
   * 
   * @param carTf 
   */
  void updateLocal(const Eigen::Affine3d &carTf);
};

#endif  // STRUCTURES_OBSERVATION_HPP