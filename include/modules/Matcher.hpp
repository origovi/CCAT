#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <vector>
#include <iostream>

#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "structures/Cone.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;
using Intrinsics = Eigen::Matrix<double, 3, 3>;
using Extrinsics = Eigen::Affine3d;

class Matcher {
 private:
  /**
   * PRIVATE CONSTRUCTOR
   */

  Matcher();

  /**
   * PRIVATE ATTRIBUTES
   */

  ros::NodeHandle *nh_;
  Params::Matcher params_;
  bool hasData_;
  std::vector<Cone> currentCones_;

  PCL::Ptr actualMap_;
  nav_msgs::Odometry::ConstPtr carLocation_;

  Extrinsics extrinsics_left_, extrinsics_right_;
  Intrinsics intrinsics_left_, intrinsics_right_;
  geometry_msgs::PoseArray::ConstPtr leftBbs_, rightBbs_;

  /* Publishers */

  image_transport::Publisher leftProjectedPub_, rightProjectedPub_;

  /**
   * PRIVATE METHODS
   */
  static void copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output);
  void locationTransformPCLs(const std::vector<PCL::Ptr> &reconstructions) const;
  void cameraTransformPCLs(const std::vector<PCL::Ptr> &transformedPCL,
                           const Extrinsics &camTf) const;
  std::vector<PCL::Ptr> reconstructedPCLs(const std::vector<Observation> &observations) const;
  cv::Point2d projectPoint(const PCLPoint &pointToProject,
                           const Intrinsics &intrinsics) const;
  void publishImage(const std::vector<PCL::Ptr> &recons,
                    const geometry_msgs::PoseArray::ConstPtr &bbs,
                    const image_transport::Publisher &imPub,
                    const Intrinsics &intrinsics) const;

 public:
  /**
   * PUBLIC DESTRUCTOR
   */

  ~Matcher();

  /* Singleton pattern */

  static Matcher &getInstance();
  Matcher(Matcher const &) = delete;
  void operator=(Matcher const &) = delete;

  /**
   * PUBLIC METHODS
   */

  /* Init */

  void init(ros::NodeHandle *const &nh, const Params::Matcher &params);

  /* Callbacks */

  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void locationAndBbsCbk(const nav_msgs::Odometry::ConstPtr &carPos,
                         const geometry_msgs::PoseArray::ConstPtr &leftDetections,
                         const geometry_msgs::PoseArray::ConstPtr &rightDetections);

  /* Functions */

  void run(const std::vector<Observation> &observations);

  /* Getters */

  const bool &hasData() const;
  const std::vector<Cone> &getCurrentCones() const;
};

#endif  // MATCHER_HPP