#include "structures/Params.hpp"

Params::Params(const ros::NodeHandle &nh) {
    /* Common */
  nh.param<int>("ccat/common/frequency", common.frequency, 10);
  nh.param<std::string>("/AS/P/ccat/common/topics/input/observations", common.topics.input.observations, "/cones/observed");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/odometry", common.topics.input.odometry, "/limovelo/state");
  nh.param<std::string>("/AS/P/ccat/common/topics/input/map", common.topics.input.map, "/map/accumulated");

  /* Preproc */
  nh.param<float>("/AS/P/ccat/preproc/cluster_dist", preproc.cluster_dist, 0.5);

  /* Matcher */
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/translation", matcher.extrinsics_left.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/left/euler_angles", matcher.extrinsics_left.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/translation", matcher.extrinsics_right.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/extrinsics/right/euler_angles", matcher.extrinsics_right.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/intrinsics/left/camera_matrix", matcher.intrinsics_left, std::vector<double>(9, 0.0));
  nh.param<std::vector<double>>("/AS/P/ccat/intrinsics/right/camera_matrix", matcher.intrinsics_right, std::vector<double>(9, 0.0));
  nh.param<std::string>("/AS/P/ccat/matcher/topics/input/left_detections", matcher.topics.input.left_detections, "/camera/left/detections");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/input/right_detections", matcher.topics.input.right_detections, "/camera/right/detections");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/left_projected", matcher.topics.output.projected_left, "/AS/P/ccat/left_projected");
  nh.param<std::string>("/AS/P/ccat/matcher/topics/output/right_projected", matcher.topics.output.projected_right, "/AS/P/ccat/right_projected");
  
  /* Tracker */
  //nh.param<float>("ccat/preproc/clusterDist", tracker.clusterDist, 0.5);

}