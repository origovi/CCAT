#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <ccat/ExtrinsicsConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <iostream>
#include <vector>

#include "structures/Cone.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "utilities/conversions.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;
using Intrinsics = Eigen::Matrix<double, 3, 4>;
using Extrinsics = Eigen::Affine3d;

class Matcher {
 private:
  struct Projection {
    size_t obsIndex;
    std::vector<cv::Point2d> projPoints;  // in image space
  };

  /**
   * PRIVATE ATTRIBUTES
   */

  ros::NodeHandle *nh_;
  const Params::Matcher params_;
  std::vector<Cone> currentCones_;

  PCL::Ptr actualMap_;
  std_msgs::Header current_header_;

  Extrinsics extrinsics_;
  const Intrinsics intrinsics_;
  geometry_msgs::PoseArray::ConstPtr bbs_;

  /* Publishers */

  image_transport::Publisher projectedPub_;

  /**
   * PRIVATE METHODS
   */
  static void copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output);
  std::list<Projection> transformPCLs(
      const std::vector<PCL::Ptr> &pcls, const Eigen::Transform<double, 3, Eigen::Projective> &tf) const;
  void cameraTransformPCLs(const std::vector<PCL::Ptr> &transformedPCL,
                           const Extrinsics &camTf) const;
  std::vector<PCL::Ptr> reconstructedPCLs(const std::vector<Observation> &observations) const;
  cv::Point2d projectPoint(const PCLPoint &pointToProject, const Intrinsics &intrinsics) const;
  void projections(const std::vector<PCL::Ptr> &recons, std::list<Projection> &projs) const;
  void publishImage2(const std::list<Projection> &projections,
                     const geometry_msgs::PoseArray::ConstPtr &bbs) const;
  void publishImage(const std::vector<PCL::Ptr> &recons,
                    const geometry_msgs::PoseArray::ConstPtr &bbs,
                    const image_transport::Publisher &imPub, const Intrinsics &intrinsics) const;
  void match(const std::list<Projection> &projections, const geometry_msgs::PoseArray::ConstPtr &bbs, std::vector<Cone> &matchings);

 public:
  enum Which { LEFT,
               RIGHT };
  struct RqdData {
    std::vector<Observation::Ptr> observations;
    geometry_msgs::PoseArray::ConstPtr bbs;
  };

  /**
   * PUBLIC CONSTRUCTOR AND DESTRUCTOR
   */

  Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, const Which &which);
  ~Matcher();

  /**
   * PUBLIC ATTRIBUTES
   */

  const Which which;

  /**
   * PUBLIC METHODS
   */

  /* Callbacks */

  void cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level);

  /* Functions */

  void run(const RqdData &data);

  /* Getters */

  const std::vector<Cone> &getData() const;
};

#endif  // MATCHER_HPP