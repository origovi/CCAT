#ifndef STRUCTURES_PARAMS_HPP
#define STRUCTURES_PARAMS_HPP

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
        std::string odom;
        std::string left_bbs, right_bbs;
      } input;
      struct {
        std::string cones;
      } output;
    } topics;
  } common;

  /**
   * MANAGER
   */

  struct Manager {
    float bufferTempMem;
    bool static_calib;
    bool publish_only_odom_update;
  } manager;

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
    double min_dist_car_able_to_match;
    int image_width, image_height;
    float cone_height, cone_width;
    bool colors_in_projected;
    std::string autocalib_service_addr;
    std::vector<float> intrinsics;
    struct {
      struct {
      } input;
      struct {
        std::string projected;
        std::string pcl;
        std::string instant_markers;
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
    bool fancy_markers, show_markers_id, markers_on_ground;
    int heap_size;
    struct {
      struct {
      } input;
      struct {
        std::string mergedMarkers;
        std::string finalMarkersBaseLink, finalMarkersGlobal;
      } output;
    } topics;
  } tracker;
};

#endif  // STRUCTURES_PARAMS_HPP