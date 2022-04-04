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

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "structures/Cone.hpp"
#include "structures/KDTree.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "utilities/conversions.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;
using Intrinsics = Eigen::Matrix3f;
using Extrinsics = Eigen::Affine3d;

class Matcher {
 private:
  /**
  * PRIVATE STRUCTURES
  */
  struct Projection {
    Observation::Ptr observation;
    std::vector<cv::Point2d> projPoints;
    cv::Point2d imageCentroid;
  };

  struct Match {
   private:
    bool valid_ = false;
    size_t bbInd_;
    double dist_;
   public:
    explicit operator bool() const { return valid_; }
    void match(size_t bbInd, double dist) {
      valid_ = true;
      bbInd_ = bbInd;
      dist_ = dist;
    }
    void unmatch() { valid_ = false; }
    const size_t &bbInd() const { return bbInd_; }
    const double &dist() const { return dist_; }
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
  ros::Publisher pclPub_;

  /**
   * PRIVATE METHODS
   */
  static void copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output);
  void projections(const std::vector<Observation::Ptr> &observations, std::vector<Projection> &projs);
  void publishPCLs(const std::vector<Observation::Ptr> &observations) const;
  void publishImage(const std::vector<Projection> &projections,
                    const geometry_msgs::PoseArray::ConstPtr &bbs) const;
  inline double bbHeightFromZ(const double &z) const;
  static Point bbCentroidAndHeight(const geometry_msgs::Pose &bb);
  void match(const size_t &bbInd, const geometry_msgs::PoseArray &bbs, const KDTree &projsKDT, std::vector<Match> &matches, std::vector<std::set<size_t>> &projsToExclude) const;
  void computeMatches(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs);

 public:
  enum Which { LEFT,
               RIGHT };
  struct RqdData {
    std::vector<Observation> observations;
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