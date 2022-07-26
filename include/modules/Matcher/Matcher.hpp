/**
 * @file Matcher.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Matcher module
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_MATCHER_MATCHER_HPP
#define MODULES_MATCHER_MATCHER_HPP

#include <ccat/CalibReq.h>
#include <ccat/ExtrinsicsConfig.h>
#include <dynamic_reconfigure/server.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "modules/Matcher/MatcherVis.hpp"
#include "modules/Matcher/Matching.hpp"
#include "structures/ConeUpdate.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "utils/KDTree.hpp"

using PCLPoint = pcl::PointXYZI;
using PCL = pcl::PointCloud<PCLPoint>;
using Intrinsics = Eigen::Matrix3f;
using Extrinsics = Eigen::Affine3d;

/**
 * @brief Module that contains all the matching pipeline. From detections from
 * the global 3D map and the camera bounding boxes (+ car's state), register
 * these variables and output the matches, i.e. cone \a id with its
 * corresponding type.
 */
class Matcher {
 public:
  /* ---------------------------- Public Structs ---------------------------- */

  /**
   * @brief Structure that allows to uniquely relate a Matcher module to its
   * correspondant camera.
   */
  enum Which { LEFT,
               RIGHT };

 private:

  /**
   * @brief Represents the data that a Projection needs
   */
  struct ProjectionData {
    /**
     * @brief The index of the correspondant Observation in the input
     * Observation vector.
     */
    size_t obsInd;

    /**
     * @brief A shared pointer to the correspondant Observation.
     * 
     */
    Observation::Ptr obs;

    /**
     * @brief The cone centroid in camera coordinates.
     */
    Point centroid_cam;

    /**
     * @brief A shared pointer to the cone point cloud in camera coords.
     */
    PCL::Ptr pcl_cam = pcl::make_shared<PCL>();
  };

  /**
   * @brief Represents a projected Observation, it contains:
   * - A pointer to the correspondant ProjectionData.
   * - All the points of the Observation in 2D in image space.
   * - The 2D centroid of the Observation pointcloud in image space.
   */
  struct Projection {

    /**
     * @brief A pointer to the correspondant ProjectionData, i.e. information
     * about the Observation it belongs.
     */
    const ProjectionData *data;

    /**
     * @brief All the points of the Observation in 2D in image space.
     */
    std::vector<cv::Point2d> projPoints;

    /**
     * @brief The 2D centroid of the Observation pointcloud in image space.
     */
    cv::Point2d imageCentroid;
  };


  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Matcher, it includes all necessary
   * paramters that will be used in the module.
   */
  const Params::Matcher params_;

  /**
   * @brief Specifies if the implicit camera is well calibrated.
   */
  bool calibrated_;

  /**
   * @brief Output of the Matcher module, in every iteration, it gets updated
   * with all cones that appear in-camera.
   */
  std::vector<ConeUpdate> currentUpdates_;

  /**
   * @brief The camera extrinsics of the camera implicit in the Matcher.
   * It is used to transform a local position into camera space.
   * It has the capability to update via a dynamic reconfigure.
   */
  Extrinsics extrinsics_;

  /**
   * @brief The camera intrinsics of the camera implicit in the Matcher.
   * It is used to transform a camera space position into image space.
   */
  const Intrinsics intrinsics_;

  /**
   * @brief The Visualization object for the Matcher, it will allow us to
   * publish all debug messages.
   */
  MatcherVis vis_;

  /**
   * @brief Server of the dynamic reconfigure for the camera extrinsics.
   * It will be used to update the parameters on autocalib.
   */
  dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfg_cam_srv;

  /* -------------------------- Private Subscribers ------------------------- */

  /**
   * @brief The service that will allow this camera to be calibrated;
   * will return the calibrated extrinsics.
   */
  ros::ServiceClient calibSrv_;

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
  void projections(const std::vector<ProjectionData> &projDatas, std::vector<Projection> &projs);

  /**
   * @brief Calculates the height that would have a YOLO-BB of an Observation
   * given its distance to the camera plane \a z.
   * 
   * @param[in] z The distance of an Observation from the camera plane in meters
   * @return double The height that would have a YOLO-BB of an Observation.
   */
  inline double bbHeightFromZ(const double &z) const;

  /**
   * @brief Specifies if a matching is possible given the distance of matching
   * (px) and the bounding box height. This is equivalent to a distance variable
   * threshold.
   * 
   * @param[in] matchingDist 
   * @param[in] bbHeight 
   * @return true 
   * @return false 
   */
  inline bool isMatchingPossible(const double &matchingDist, const double &bbHeight) const;

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
   * @param[out] matches A vector of Matching(s)
   * @param[out] projsToExclude For each BB, all the Matcher::Projection(s)
   * it cannot be matched to
   * @param[in,out] match_num The number of successful matches
   */
  void matchBestFit(const size_t &bbInd, const std::vector<geometry_msgs::Pose> &bbs, const KDTree &projsKDT, std::vector<Matching> &matches, std::vector<std::set<size_t>> &projsToExclude, int &match_num) const;

  /**
   * @brief Matches the Matcher::Projection with index = \a projInd with the
   * closest BB, that means that each BB can be matched more than once.
   * The computational complexity is O(logn) as it uses a KDTree.
   * 
   * @param[in] projInd The index of the Matcher::Projection it is gonna find a
   * matching for
   * @param[in] projections A vector of all Matcher::Projection(s)
   * @param[in] bbsKDT A KDTree of every BB in the image
   * @param[out] matches A vector of Matching(s)
   * @param[in,out] match_num The number of successful matches
   */
  void matchGreedy(const size_t &projInd, const std::vector<Projection> &projections, const KDTree &bbsKDT, std::vector<Matching> &matches, int &match_num) const;

  /**
   * @brief Computes all the matchings, from the Matcher::Projections that are
   * in front of the camera and all the BB(s).
   * The result will be placed directly in the currentUpdates_ attribute.
   * 
   * @param[in] projections All the Matcher::Projections
   * @param[in] bbs All the BB(s)
   * @param[in,out] match_num The number of successful matches
   */
  void computeMatchings(const std::vector<Projection> &projections, const std::vector<geometry_msgs::Pose> &bbs, std::vector<Matching> &matchings, int &match_num);

  /**
   * @brief Updates the currentUpdates_ attribute.
   * 
   * @param[in] projs
   * @param[in] projDatas 
   * @param[in] bbs All the BB(s)
   * @param[in] matchings Current iteration's mathings (relation BB-Projection)
   */
  void updateData(const std::vector<Projection> &projs, const std::vector<ProjectionData> &projDatas, const geometry_msgs::PoseArray &bbs, const std::vector<Matching> &matchings);

  /**
   * @brief Calls the autocalibration service with current matches.
   * Updates the extrinsics_ parameters if the calibration has been successful.
   * 
   * @param[in] projections 
   * @param[in] bbs All the BB(s)
   * @param[in] matchings Current iteration's mathings (relation BB-Projection)
   */
  void autocalib(const std::vector<Projection> &projections, const geometry_msgs::PoseArray &bbs, const std::vector<Matching> &matchings);

  /**
   * @brief Constructs and returns a Point containing the BB centroid
   * (image space) and the height of the BB in pixels in z.
   * 
   * @param[in] bb 
   * @return A Point that will contain (image space):
   * - x: the BB centroid x coordinate
   * - y: the BB centroid y coordinate
   * - z: the BB height
   */
  static Point bbCentroidAndHeight(const geometry_msgs::Pose &bb);

 public:
  /* --------------------------- Public Constructor -------------------------- */

  /**
   * @brief Construct a new Matcher object
   * 
   * @param[in] params 
   * @param[in] nh 
   * @param[in] cfg_cam_srv 
   * @param[in] which 
   */
  Matcher(const Params::Matcher &params, ros::NodeHandle *const &nh, dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfg_cam_srv, const Which &which);

  /* --------------------------- Public Attributes -------------------------- */

  /**
   * @brief Specifies which camera is the Matcher linked to
   */
  const Which which;

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Dynamic Reconfigure callback function for the extrinsics parameters.
   * It replaces the actual Extrinsics with the new ones.
   * Useful for calibration.
   * 
   * @param[in] config 
   * @param[in] level 
   */
  void cfgCallback(const ccat::ExtrinsicsConfig &config, uint32_t level);

  /**
   * @brief Main function of the Matcher class, it runs all the pipeline and
   * updates the currentCone_ attribute as a result.
   * 
   * @param[in] observations 
   * @param[in] bbs 
   */
  void run(const std::vector<Observation::Ptr> &observations, const geometry_msgs::PoseArray::ConstPtr &bbs);

  /**
   * @brief Returns the currentUpdates_ attribute, which contains the result of
   * the Matcher pipeline.
   * 
   * @return The updates of cones computed in the last iteration of
   * Matcher::run() method
   */
  const std::vector<ConeUpdate> &getData() const;
};

#endif  // MODULES_MATCHER_MATCHER_HPP