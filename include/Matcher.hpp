#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "Observation.hpp"
#include "Params.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;

class Matcher {
 private:
  /**
   * CONSTRUCTORS
   */
  Matcher(const Params::Matcher &params);

  /**
   * DESTRUCTORS
   */
  ~Matcher();

  /**
   * PRIVATE ATTRIBUTES
   */
  const Params::Matcher params_;
  bool hasData_;

  PCL::Ptr actualMap_;

  nav_msgs::Odometry::ConstPtr carLocation_;
  geometry_msgs::PoseArray::ConstPtr leftDetections_;
  geometry_msgs::PoseArray::ConstPtr rightDetections_;

  /**
   * PRIVATE METHODS
   */
  std::vector<PCL::Ptr> reconstructions(const std::vector<Observation> &observations) const;

 public:
  /**
   * PUBLIC METHODS
   */
  /* Singleton pattern */
  static Matcher &getInstance(const Params::Matcher &params);
  Matcher(Matcher const &) = delete;
  void operator=(Matcher const &) = delete;

  /* Callbacks */
  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void locationAndBbsCbk(const nav_msgs::Odometry::ConstPtr &carPos, const geometry_msgs::PoseArray::ConstPtr &leftDetections, const geometry_msgs::PoseArray::ConstPtr &rightDetections);

  /* Functions */
  void run(const std::vector<Observation> &observations);

  /* Getters */
  const bool &hasData() const;
};

#endif  // MATCHER_HPP