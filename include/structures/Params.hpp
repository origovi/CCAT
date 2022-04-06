#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

struct Params {

  /**
   * CONSTRUCTOR
   */

  Params(const ros::NodeHandle &nh);
  
  /**
   * COMMON
   */
  
  struct Common {
    int frequency;
    struct {
      struct {
        std::string observations;
        std::string odometry;
        std::string map;
        std::string left_bbs, right_bbs;
      } input;
      struct {
        std::string cones;
      } output;
    } topics;
  } common;

  /**
   * PREPROC
   */

  struct Preproc {
    float cluster_dist;
    struct {
      struct {
      } input;
      struct {
      } output;
    } topics;
  } preproc;

  /**
   * MATCHER
   */

  struct Matcher {
    bool debug;
    std::string match_type;
    double max_match_search_dist;
    double max_match_real_dist;
    int image_width, image_height;
    float cone_height, cone_width;
    struct {
      std::vector<double> translation;
      std::vector<double> euler_angles;
    } extrinsics;
    std::vector<float> intrinsics;
    struct {
      struct {
      } input;
      struct {
        std::string projected;
        std::string pcl;
      } output;
    } topics;
  } matcherL, matcherR;

  /**
   * MERGER
   */

  struct Merger {

  } merger;

  /**
   * TRACKER
   */

  struct Tracker {
    bool debug;
    double same_cone_max_distSq;
    bool fancy_markers;
    struct {
      struct {
      } input;
      struct {
        std::string markersBaseLink, markersGlobal;
      } output;
    } topics;
  } tracker;
};

#endif