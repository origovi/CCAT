/**
 * @file MatcherVis.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It provides all the necessary tools to the Matcher module to
 * visualize all the data and debug.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_MATCHER_MATCHERVIS_HPP
#define MODULES_MATCHER_MATCHERVIS_HPP

#include "structures/Params.hpp"
#include "utils/Visualization.hpp"
#include "modules/Matcher/Matching.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include "structures/Observation.hpp"

/**
 * @brief Visualization class for the Matcher module.
 * 
 */
class MatcherVis : public Visualization {
 private:
  /* -------------------------- Private Attributes -------------------------- */
  
  const Params::Matcher params_;
  
  /* ---------------------------- Private Methods --------------------------- */

  /**
   * @brief Transform the type of a bounding box into a std_msgs::ColorRGBA
   * 
   * @param[in] BBColor the type of cone of a bounding box
   * @return a color in std_msgs format
   */
  static std_msgs::ColorRGBA BBColorToStd(const double &BBColor);

  /**
   * @brief Transform the type of a bounding box into a cv::Scalar, useful
   * when using OpenCV to display images.
   * 
   * @param[in] BBColor the type of cone of a bounding box
   * @return a color in OpenCV format
   */
  static cv::Scalar BBColorToCV(const double &BBColor);

 public:
  /* -------------------------- Public Constructor -------------------------- */

  MatcherVis(const Params::Matcher &params);

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief For debug purposes, it publishes the PCL(s) corresponding to all
   * Observation(s) in camera space. The result will be published as a
   * sensor_msgs::PointCloud2.
   * 
   * @param[in] observations 
   */
  void publishPCLs(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &pcls);

  /**
   * @brief For debug purposes, it publishes the projected image of all the
   * Observation(s) that the camera sees with its corresponding centroid AND
   * all the BB(s).
   * 
   * @param[in] projections 
   * @param[in] bbs 
   * @param[in] matchings 
   */
  void publishProjectedImg(const std::vector<std::pair<std::vector<cv::Point2d>, cv::Point2d>> &projections,
                           const geometry_msgs::PoseArray &bbs,
                           const std::vector<Matching> &matchings);
  
  /**
   * @brief For debug purposes, it publishes the markers of this iteration's
   * matches, it is very useful to see which cones get matched with certain
   * BBs.
   * 
   * @param[in] projsCentroids 
   * @param[in] bbs 
   * @param[in] matchings 
   */
  void publishMatchingMarkers(const std::vector<geometry_msgs::Point> &projsCentroids,
                           const geometry_msgs::PoseArray &bbs,
                           const std::vector<Matching> &matchings);
};

#endif  // MODULES_MATCHER_MATCHERVIS_HPP