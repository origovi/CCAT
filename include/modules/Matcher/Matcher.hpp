/**
 * @file Matcher.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Matcher module
 * @version 1.0
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <ccat/ExtrinsicsConfig.h>
#include <ccat/CalibReq.h>
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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;
using Intrinsics = Eigen::Matrix3f;
using Extrinsics = Eigen::Affine3d;

/**
 * @brief Module that contains all the matching pipeline.
 * 
 */
class Matcher {
 private:
  ros::Publisher nose_;
  /* ---------------------------- Private Structs --------------------------- */

  /**
   * @brief Represents a projected Observation, it contains:
   * - A pointer to the Observation it represents.
   * - All the points of the Observation in 2D in image space.
   * - The 2D centroid of the Observation pointcloud in image space.
   */
  struct Projection {
    /**
     * @brief A std::shared_ptr pointer to the Observation it represents.
     */
    Observation::Ptr observation;

    /**
     * @brief All the points of the Observation in 2D in image space.
     */
    std::vector<cv::Point2d> projPoints;

    /**
     * @brief The 2D centroid of the Observation pointcloud in image space.
     */
    cv::Point2d imageCentroid;
  };

  /**
   * @brief Represents a Matching between a bounding box and an Observation.
   * 
   * Usually in a vector of Matching(s), element \a i specifies that
   * Observation \a i is matching to the BB with index = \a bbInd_.
   */
  struct Matching {
   private:
    /**
    * @brief Whether or not this match is valid. Initialized to false.
    */
    bool valid_ = false;

    /**
     * @brief The BB index this Observation is matched to.
     */
    size_t bbInd_;

    /**
     * @brief The matching distance between the BB and the Observation.
     */
    double dist_;

   public:
    /**
    * @brief Getter to know if the matching is valid or not.
    * 
    * @return true if the matching is valid.
    * @return false if the matching is not valid.
    */
    explicit operator bool() const { return valid_; }

    /**
     * @brief Match the implicit object to the BB with index \a bbInd and with
     * a matching distance \a dist.
     * 
     * @param bbInd Index of the BB
     * @param dist Matching distance
     */
    void match(size_t bbInd, double dist) {
      valid_ = true;
      bbInd_ = bbInd;
      dist_ = dist;
    }

    /**
     * @brief Invalidates the matching.
     */
    void unmatch() { valid_ = false; }

    /**
     * @brief Getter for \a bbInd_.
     * 
     * @return The index of the matched BB.
     * note: if not \a valid_, the result of this method is invalid as well.
     */
    const size_t &bbInd() const { return bbInd_; }

    /**
     * @brief Getter for \a dist_.
     * 
     * @return The matching distance between the implicit object and
     * the BB with index = \a bbInd_.
     * note: if not \a valid_, the result of this method is invalid as well.
     */
    const double &dist() const { return dist_; }
  };

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Matcher.
   */
  const Params::Matcher params_;

  /**
   * @brief Specifies if the currentCones_ attribute data is valid or not.
   * 
   */
  bool hasValidData_;

  /**
   * @brief Specifies if the implicit camera is well calibrated.
   * 
   */
  bool calibrated_;

  /**
   * @brief The result of matching the BB(s) with the Observations
   * of last iteration.
   */
  std::vector<Cone> currentCones_;

  /**
   * @brief The camera extrinsics of the camera implicit in the Matcher.
   * It is used to transform a base_link position into camera space.
   * It has the capability to update via a dynamic reconfigure.
   */
  Extrinsics extrinsics_;

  /**
   * @brief The camera intrinsics of the camera implicit in the Matcher.
   * It is used to transform a camera space position into image space.
   */
  const Intrinsics intrinsics_;

  /* -------------------------- Private Subscribers ------------------------- */
  
  /**
   * @brief The service that will allow this camera to be calibrated;
   * will return the calibrated extrinsics.
   * 
   */
  ros::ServiceClient calibSrv_;

  /* -------------------------- Private Publishers -------------------------- */

  /**
   * @brief Publisher used to advertise the projected Observations of the
   * implicit camera.
   */
  image_transport::Publisher projectedPub_;

  /**
   * @brief Publisher used to advertise the transformed pointcloud
   * in camera space. For debug purposes only.
   * 
   */
  ros::Publisher pclPub_;

  /* ---------------------------- Private Methods --------------------------- */

  /**
   * @brief It projects all Observation(s) points into the camera and puts
   * the ones that can be seen (in front of camera) into the projs parameter.
   * 
   * @param[in] observations Contains all the Observation(s)
   * @param[out] projs Will contain all the Matcher::Projection(s) that the
   * camera sees (at least one point in front of the camera),
   * in image coordinates
   */
  void projections(const std::vector<Observation::Ptr> &observations, std::vector<Projection> &projs);

  /**
   * @brief Calculates the height that would have a YOLO-BB of an Observation
   * given its distance to the camera plane \a z.
   * 
   * @param z The distance of an Observation from the camera plane in meters
   * @return double The height that would have a YOLO-BB of an Observation.
   */
  inline double bbHeightFromZ(const double &z) const;

  /**
   * @brief Matches the BB with index = \a bbInd with the closest
   * Matcher::Projection in such a way that each BB is matched only once.
   * It starts with a BB \a bb0, if the closest Matcher::Projection \a proj does
   * not have a matching, it matches \a bb0 with \a proj. Otherwise \a proj is
   * matched to \a bb1, then it compares the distance of the \a proj - \a bb0
   * matching with the matching distance of \a proj - \a bb1, if it is smaller it
   * matches \a bb0 to \a proj, unmatches \a bb1, adds \a proj to the exclusion
   * list of \a bb1 and finds another matching for \a bb1. Recursive.
   * 
   * @param[in] bbInd The index of the BB it is gonna find a matching for
   * @param[in] bbs An array of all BB(s)
   * @param[in] projsKDT A KDTree of every Matcher::Projection in the image
   * @param[out] matches A vector of Matcher::Matching
   * @param[out] projsToExclude For each BB, all the Matcher::Projection(s)
   * it cannot be matched to
   */
  void matchBestFit(const size_t &bbInd, const geometry_msgs::PoseArray &bbs, const KDTree &projsKDT, std::vector<Matching> &matches, std::vector<std::set<size_t>> &projsToExclude) const;
  
  /**
   * @brief Matches the Matcher::Projection with index = \a projInd with the
   * closest BB, that means that each BB can be matched more than once.
   * The computational complexity is O(logn) as it uses a KDTree.
   * 
   * @param[in] projInd The index of the Matcher::Projection it is gonna find a
   * matching for
   * @param[in] projections A vector of all Matcher::Projection(s)
   * @param[in] bbsKDT A KDTree of every BB in the image
   * @param[out] matches A vector of Matcher::Matching
   */
  void matchGreedy(const size_t &projInd, const std::vector<Projection> &projections, const KDTree &bbsKDT, std::vector<Matching> &matches) const;
  
  /**
   * @brief Computes all the matchings, from the Matcher::Projections that are
   * in front of the camera and all the BB(s).
   * The result will be placed directly in the currentCones_ attribute.
   * 
   * @param[in] projections All the Matcher::Projections
   * @param[in] bbs All the BB(s)
   */
  void computeMatches(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs);
  
  /**
   * @brief For debug purposes, it publishes the PCL(s) corresponding to all
   * Observation(s) in camera space. The result will be published as a
   * sensor_msgs::PointCloud2.
   * 
   * @param observations All the Observation(s)
   */
  void publishPCLs(const std::vector<Observation::Ptr> &observations) const;

  /**
   * @brief For debug purposes, it publishes the projected image of all the
   * Observation(s) that the camera sees with its corresponding centroid AND
   * all the BB(s).
   * 
   * @param projections 
   * @param bbs 
   */
  void publishImage(const std::vector<Projection> &projections, const geometry_msgs::PoseArray::ConstPtr &bbs) const;

  /**
   * @brief It deep copies a vector of pcl::shared_ptr to pcl::PointCloud.
   * 
   * @param input 
   * @param output 
   */
  static void copyPCLPtrVec(const std::vector<PCL::Ptr> &input, std::vector<PCL::Ptr> &output);

  /**
   * @brief Constructs and returns a Point containing the BB centroid
   * (image space) and the height of the BB in pixels in z.
   * 
   * @param bb 
   * @return A Point that will contain (image space):
   * - x: the BB centroid x coordinate
   * - y: the BB centroid y coordinate
   * - z: the BB height
   */
  static Point bbCentroidAndHeight(const geometry_msgs::Pose &bb);

 public:
  /* ---------------------------- Public Structs ---------------------------- */

  enum Which { LEFT,
               RIGHT };

  /**
   * @brief Encapsulates the data needed to run the Matcher pipeline.
   * 
   */
  struct RqdData {
    std::vector<Observation> observations;
    geometry_msgs::PoseArray::ConstPtr bbs;
  };

  /* --------------------------- Public Constructor -------------------------- */

  /**
   * @brief Construct a new Matcher object
   * 
   * @param params 
   * @param nh 
   * @param which 
   */
  Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, const Which &which);

  /* --------------------------- Public Attributes -------------------------- */

  /**
   * @brief Specifies which camera is the Matcher linked to
   * 
   */
  const Which which;

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Dynamic Reconfigure callback function for the extrinsics parameters.
   * It replaces the actual Extrinsics with the new ones.
   * Useful for calibration.
   * 
   * @param config 
   * @param level 
   */
  void cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level);

  /**
   * @brief Main function of the Matcher class, it runs all the pipeline and
   * updates the currentCone_ attribute as a result.
   * 
   * @param data The data necessary (aka Observation(s) and BB(s))
   */
  void run(const RqdData &data);

  /**
   * @brief Returns the currentCones_ attribute, which contains the result of
   * the matching pipeline.
   * 
   * @return The new cones computed in the last iteration of Matcher::run() method
   */
  const std::vector<Cone> &getData() const;

  /**
   * @brief Returns whether or not this Matcher has valid data to pass to the
   * next module (which is Merger).
   * 
   * @return true 
   * @return false 
   */
  const bool &hasValidData() const;
};

#endif  // MATCHER_HPP