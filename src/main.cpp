#include <geometry_msgs/PoseArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "Matcher.hpp"
#include "Preproc.hpp"
#include "Tracker.hpp"

void loadParams(Params &params, const ros::NodeHandle &nh) {
  /* Common */
  nh.param<int>("urimap/common/frequency", params.common.frequency, 10);

  /* Topics */
  nh.param<std::string>("urimap/topics/input/observations", params.common.topics.input.observations, "/cones/observed");
  nh.param<std::string>("urimap/topics/input/map", params.common.topics.input.map, "/map/accumulated");

  /* Preproc */
  nh.param<float>("urimap/preproc/cluster_dist", params.preproc.cluster_dist, 0.5);

  /* Matcher */
  nh.param<std::vector<double>>("urimap/extrinsics/left/translation", params.matcher.extrinsics_left.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("urimap/extrinsics/left/euler_angles", params.matcher.extrinsics_left.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("urimap/extrinsics/right/translation", params.matcher.extrinsics_right.translation, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("urimap/extrinsics/right/euler_angles", params.matcher.extrinsics_right.euler_angles, std::vector<double>(3, 0.0));
  nh.param<std::vector<double>>("urimap/intrinsics/left", params.matcher.intrinsics_left, std::vector<double>(9, 0.0));
  nh.param<std::vector<double>>("urimap/intrinsics/right", params.matcher.intrinsics_right, std::vector<double>(9, 0.0));
  
  /* Tracker */
  //nh.param<float>("urimap/preproc/clusterDist", params.tracker.clusterDist, 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "urimap");
  ros::NodeHandle *const nh(new ros::NodeHandle);

  Params params;
  loadParams(params, *nh);

  // Object instances
  Preproc &preproc = Preproc::getInstance();
  preproc.init(nh, params.preproc);
  Matcher &matcher = Matcher::getInstance();
  matcher.init(nh, params.matcher);
  Tracker &tracker = Tracker::getInstance();
  tracker.init(nh, params.tracker);

  // Subscribers & Publisher
  ros::Subscriber subPau = nh->subscribe("/cones/observed", 1, &Preproc::obsCallback, &preproc);
  ros::Subscriber subMap = nh->subscribe("/map/observed", 1, &Matcher::mapCallback, &matcher);

  message_filters::Subscriber<nav_msgs::Odometry> state_carSub(*nh, "/state/car", 100);

  // cameras detections
  message_filters::Subscriber<geometry_msgs::PoseArray> left_detectionsSub(*nh, "/camera/left/detections", 100);
  message_filters::Subscriber<geometry_msgs::PoseArray> right_detectionsSub(*nh, "/camera/right/detections", 100);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseArray, geometry_msgs::PoseArray> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> syncDetections(MySyncPolicy(100), state_carSub, left_detectionsSub, right_detectionsSub);
  syncDetections.registerCallback(boost::bind(&Matcher::locationAndBbsCbk, &matcher, _1, _2, _3));

  ros::Rate rate(params.common.frequency);
  while (ros::ok()) {
    ros::spinOnce();
    if (preproc.hasData() and matcher.hasData()) {
      matcher.run(preproc.getCurrentObservations());
    }
    rate.sleep();
  }
  delete nh;
}