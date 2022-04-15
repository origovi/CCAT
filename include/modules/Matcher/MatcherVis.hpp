#ifndef MODULES_MATCHER_MATCHERVIS_HPP
#define MODULES_MATCHER_MATCHERVIS_HPP

#include "structures/Params.hpp"
#include "utilities/Visualization.hpp"
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
  const Params::Matcher params_;
  static std_msgs::ColorRGBA BBColorToStd(const double &BBColor);
  static cv::Scalar BBColorToCV(const double &BBColor);

 public:
  MatcherVis(ros::NodeHandle *const nh, const Params::Matcher &params);

  /**
   * @brief For debug purposes, it publishes the PCL(s) corresponding to all
   * Observation(s) in camera space. The result will be published as a
   * sensor_msgs::PointCloud2.
   * 
   * @param observations 
   */
  void publishPCLs(const std::vector<Observation::Ptr> &observations);

  /**
   * @brief For debug purposes, it publishes the projected image of all the
   * Observation(s) that the camera sees with its corresponding centroid AND
   * all the BB(s).
   * 
   * @param projections 
   * @param bbs 
   * @param matchings 
   */
  void publishProjectedImg(const std::vector<std::pair<std::vector<cv::Point2d>, cv::Point2d>> &projections,
                           const geometry_msgs::PoseArray &bbs,
                           const std::vector<Matching> &matchings);
  void publishMatchingMarkers(const std::vector<geometry_msgs::Point> &projsCentroids,
                           const geometry_msgs::PoseArray &bbs,
                           const std::vector<Matching> &matchings);
};

#endif  // MODULES_MATCHER_MATCHERVIS_HPP