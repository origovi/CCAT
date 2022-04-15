#include "structures/Params.hpp"

Params::Params(const ros::NodeHandle &nh) {
  /**
   * COMMON
   */

  nh.param<int>("/AS/P/ccat/common/frequency", common.frequency, 10);
  nh.param<std::string>("/AS/P/ccat/common/topics/input/observations", common.topics.input.observations, "/cones/observed");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/odometry", common.topics.input.odometry, "/limovelo/state");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/map", common.topics.input.map, "/map/accumulated");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/left_detections", common.topics.input.left_bbs, "/camera/left/detections");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/right_detections", common.topics.input.right_bbs, "/camera/right/detections");
  nh.param<std::string>("/AS/P/ccat/common/topics/output/cones", common.topics.output.cones, "/AS/P/ccat/cones");

  /**
   * PREPROC
   */

  nh.param<float>("/AS/P/ccat/preproc/cluster_dist", preproc.cluster_dist, 0.5);

  /**
   * MATCHER
   */

  // Common Part
  nh.param<bool>("/AS/P/ccat/matcher/common/debug", matcherL.debug, false);
  nh.param<std::string>("/AS/P/ccat/matcher/common/match_type", matcherL.match_type, "greedy");
  nh.param<double>("/AS/P/ccat/matcher/common/max_match_search_dist", matcherL.max_match_search_dist, 50.0);
  nh.param<double>("/AS/P/ccat/matcher/common/max_match_real_dist", matcherL.max_match_real_dist, 50.0);
  nh.param<int>("/AS/P/ccat/matcher/common/image_width", matcherL.image_width, 1024);
  nh.param<int>("/AS/P/ccat/matcher/common/image_height", matcherL.image_height, 768);
  nh.param<float>("/AS/P/ccat/matcher/common/cone_width", matcherL.cone_width, 0.228);
  nh.param<float>("/AS/P/ccat/matcher/common/cone_height", matcherL.cone_height, 0.325);
  nh.param<bool>("/AS/P/ccat/matcher/common/colors_in_projected", matcherL.colors_in_projected, false);
  nh.param<std::string>("/AS/P/ccat/matcher/common/service_addr", matcherL.service_addr, "/AS/P/calibration");
  matcherR = matcherL;

  // Different Part
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_projected", matcherL.topics.output.projected, "/AS/P/ccat/projected/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_projected", matcherR.topics.output.projected, "/AS/P/ccat/projected/right");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_pcl", matcherL.topics.output.pcl, "/AS/P/ccat/pcl/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_pcl", matcherR.topics.output.pcl, "/AS/P/ccat/pcl/right");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_matched_markers", matcherL.topics.output.matched_markers, "/AS/P/ccat/markers/matchings/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_matched_markers", matcherR.topics.output.matched_markers, "/AS/P/ccat/markers/matchings/right");
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/translation", matcherL.extrinsics.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/euler_angles", matcherL.extrinsics.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/translation", matcherR.extrinsics.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/euler_angles", matcherR.extrinsics.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<float>>("/AS/P/ccat/intrinsics/left/camera_matrix", matcherL.intrinsics, std::vector<float>(9, 0.0));
  nh.param<std::vector<float>>("/AS/P/ccat/intrinsics/right/camera_matrix", matcherR.intrinsics, std::vector<float>(9, 0.0));
  
  /* Tracker */
  nh.param<bool>("/AS/P/ccat/tracker/debug", tracker.debug, false);
  nh.param<double>("/AS/P/ccat/tracker/same_cone_max_distSq", tracker.same_cone_max_distSq, 0.09);
  nh.param<bool>("/AS/P/ccat/tracker/fancy_markers", tracker.fancy_markers, false);
  nh.param<std::string>("/AS/P/ccat/tracker/topics/output/markers_base_link", tracker.topics.output.markersBaseLink, "/AS/P/ccat/markers/base_link");
  nh.param<std::string>("/AS/P/ccat/tracker/topics/output/markers_global", tracker.topics.output.markersGlobal, "/AS/P/ccat/markers/global");

}