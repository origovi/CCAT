#include "structures/Params.hpp"

Params::Params(const ros::NodeHandle &nh) {

  /* ------------------------------------------------------------------------ */
  /*                                  COMMON                                  */
  /* ------------------------------------------------------------------------ */

  nh.param<std::string>("/AS/P/ccat/common/topics/input/observations", common.topics.input.observations, "/cones/observed");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/odom", common.topics.input.odom, "/limovelo/state");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/left_detections", common.topics.input.left_bbs, "/camera/left/detections");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/right_detections", common.topics.input.right_bbs, "/camera/right/detections");
  nh.param<std::string>("/AS/P/ccat/common/topics/output/cones", common.topics.output.cones, "/AS/P/ccat/cones");


  /* ------------------------------------------------------------------------ */
  /*                                  MANAGER                                 */
  /* ------------------------------------------------------------------------ */
  
  nh.param<float>("/AS/P/ccat/manager/buffer_temp_mem", manager.bufferTempMem, 1.0);
  nh.param<bool>("/AS/P/ccat/manager/only_lidar", manager.only_lidar, false);
  nh.param<bool>("/AS/P/ccat/manager/static_calib", manager.static_calib, false);
  nh.param<bool>("/AS/P/ccat/manager/publish_only_odom_update", manager.publish_only_odom_update, false);
  
  /* ------------------------------------------------------------------------ */
  /*                                  PREPROC                                 */
  /* ------------------------------------------------------------------------ */

  nh.param<float>("/AS/P/ccat/preproc/cluster_dist", preproc.cluster_dist, 0.5);
  nh.param<bool>("/AS/P/ccat/preproc/publish_markers", preproc.publish_markers, false);
  nh.param<std::string>("/AS/P/ccat/preproc/topics/output/input_markers", preproc.topics.output.input_markers, "/AS/P/ccat/markers/input");

  /* ------------------------------------------------------------------------ */
  /*                                  MATCHER                                 */
  /* ------------------------------------------------------------------------ */

  // Common Part
  nh.param<bool>("/AS/P/ccat/matcher/common/debug", matcherL.debug, false);
  nh.param<std::string>("/AS/P/ccat/matcher/common/match_type", matcherL.match_type, "greedy");
  nh.param<double>("/AS/P/ccat/matcher/common/max_match_search_dist", matcherL.max_match_search_dist, 50.0);
  nh.param<double>("/AS/P/ccat/matcher/common/min_dist_car_able_to_match", matcherL.min_dist_car_able_to_match, 5.0);
  nh.param<int>("/AS/P/ccat/matcher/common/image_width", matcherL.image_width, 1024);
  nh.param<int>("/AS/P/ccat/matcher/common/image_height", matcherL.image_height, 768);
  nh.param<float>("/AS/P/ccat/matcher/common/cone_width", matcherL.cone_width, 0.228);
  nh.param<float>("/AS/P/ccat/matcher/common/cone_height", matcherL.cone_height, 0.325);
  nh.param<bool>("/AS/P/ccat/matcher/common/colors_in_projected", matcherL.colors_in_projected, false);
  nh.param<bool>("/AS/P/ccat/matcher/common/autocalib", matcherL.autocalib, false);
  nh.param<std::string>("/AS/P/ccat/matcher/common/autocalib_service_addr", matcherL.autocalib_service_addr, "/AS/P/calibration");
  nh.param<int>("/AS/P/ccat/matcher/common/min_calib_match_num", matcherL.min_calib_match_num, 5);
  nh.param<float>("/AS/P/ccat/matcher/common/max_calib_change_trans", matcherL.max_calib_change_trans, 1e-3);
  nh.param<float>("/AS/P/ccat/matcher/common/max_calib_change_rot", matcherL.max_calib_change_rot, 1e-4);
  matcherR = matcherL;

  // Different Part
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_projected", matcherL.topics.output.projected, "/AS/P/ccat/projected/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_projected", matcherR.topics.output.projected, "/AS/P/ccat/projected/right");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_pcl", matcherL.topics.output.pcl, "/AS/P/ccat/pcl/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_pcl", matcherR.topics.output.pcl, "/AS/P/ccat/pcl/right");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_instant_markers", matcherL.topics.output.instant_markers, "/AS/P/ccat/markers/instant/left");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_instant_markers", matcherR.topics.output.instant_markers, "/AS/P/ccat/markers/instant/right");
  nh.param<std::vector<float>>("/AS/P/ccat/intrinsics/left/camera_matrix", matcherL.intrinsics, std::vector<float>(9, 0.0));
  nh.param<std::vector<float>>("/AS/P/ccat/intrinsics/right/camera_matrix", matcherR.intrinsics, std::vector<float>(9, 0.0));
  
  /* ------------------------------------------------------------------------ */
  /*                                  TRACKER                                 */
  /* ------------------------------------------------------------------------ */

  nh.param<bool>("/AS/P/ccat/tracker/debug", tracker.debug, false);
  nh.param<double>("/AS/P/ccat/tracker/same_cone_max_distSq", tracker.same_cone_max_distSq, 0.09);
  nh.param<bool>("/AS/P/ccat/tracker/fancy_markers", tracker.fancy_markers, false);
  nh.param<bool>("/AS/P/ccat/tracker/show_markers_id", tracker.show_markers_id, false);
  nh.param<bool>("/AS/P/ccat/tracker/markers_on_ground", tracker.markers_on_ground, true);
  nh.param<int>("/AS/P/ccat/tracker/cone/heap_size", tracker.cone.heap_size, 10);
  nh.param<double>("/AS/P/ccat/tracker/cone/dist_cp_to_false_positives", tracker.cone.dist_cp_to_false_positives, 15.0);
  nh.param<std::string>("/AS/P/ccat/tracker/topics/output/markers_baseLink", tracker.topics.output.mergedMarkers, "/AS/P/ccat/markers/instant/merged");
  nh.param<std::string>("/AS/P/ccat/tracker/topics/output/markers_baseLink", tracker.topics.output.finalMarkersBaseLink, "/AS/P/ccat/markers/final/base_link");
  nh.param<std::string>("/AS/P/ccat/tracker/topics/output/markers_global", tracker.topics.output.finalMarkersGlobal, "/AS/P/ccat/markers/final/global");

}