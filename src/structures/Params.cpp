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

  /**
   * PREPROC
   */

  nh.param<float>("/AS/P/ccat/preproc/cluster_dist", preproc.cluster_dist, 0.5);

  /**
   * MATCHER
   */

  // Common Part
  nh.param<int>("/AS/P/ccat/matcher/common/image_width", matcherL.image_width, 1024);
  nh.param<int>("/AS/P/ccat/matcher/common/image_height", matcherL.image_height, 768);
  matcherR = matcherL;

  // Different Part
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_projected", matcherL.topics.output.projected, "/AS/P/ccat/left_projected");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_projected", matcherR.topics.output.projected, "/AS/P/ccat/right_projected");
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/translation", matcherL.extrinsics.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/euler_angles", matcherL.extrinsics.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/translation", matcherR.extrinsics.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/euler_angles", matcherR.extrinsics.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/intrinsics/left/camera_matrix", matcherL.intrinsics, std::vector<double>(12, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/intrinsics/right/camera_matrix", matcherR.intrinsics, std::vector<double>(12, 0.0));
  
  /* Tracker */
  //nh.param<float>("ccat/preproc/clusterDist", tracker.clusterDist, 0.5);

}