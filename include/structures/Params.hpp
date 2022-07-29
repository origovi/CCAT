/**
 * @file Params.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Params class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef STRUCTURES_PARAMS_HPP
#define STRUCTURES_PARAMS_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

struct Params {

  /* -------------------------- Public Constructor -------------------------- */

  Params(const ros::NodeHandle &nh);
  
  /* ------------------------------------------------------------------------ */
  /*                                  COMMON                                  */
  /* ------------------------------------------------------------------------ */
  
  struct Common {
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

  /* ------------------------------------------------------------------------ */
  /*                                  MANAGER                                 */
  /* ------------------------------------------------------------------------ */

  struct Manager {
    float bufferTempMem;
    bool only_lidar;
    bool static_calib;
    bool publish_only_odom_update;
  } manager;

  /* ------------------------------------------------------------------------ */
  /*                                  PREPROC                                 */
  /* ------------------------------------------------------------------------ */

  struct Preproc {
    float cluster_dist;
    bool publish_markers;
    struct {
      struct {
      } input;
      struct {
        std::string input_markers;
      } output;
    } topics;
  } preproc;

  /* ------------------------------------------------------------------------ */
  /*                                  MATCHER                                 */
  /* ------------------------------------------------------------------------ */

  struct Matcher {
    bool debug;
    std::string match_type;
    double max_match_search_dist;
    double min_dist_car_able_to_match;
    int image_width, image_height;
    float cone_height, cone_width;
    bool colors_in_projected;
    bool autocalib;
    std::string autocalib_service_addr;
    int min_calib_match_num;
    float max_calib_change_trans, max_calib_change_rot;
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

  /* ------------------------------------------------------------------------ */
  /*                                  MERGER                                  */
  /* ------------------------------------------------------------------------ */

  struct Merger {

  } merger;

  /* ------------------------------------------------------------------------ */
  /*                                  TRACKER                                 */
  /* ------------------------------------------------------------------------ */

  struct Tracker {
    bool debug;
    double same_cone_max_distSq;
    bool fancy_markers, show_markers_id, markers_on_ground;
    struct Cone {
      int heap_size;
      double dist_cp_to_false_positives;
    } cone;
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