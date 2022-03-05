#include <ros/ros.h>

#include "Preproc.hpp"
#include "Matcher.hpp"
#include "Tracker.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

void loadParams(Params &params, const ros::NodeHandle &nh) {
  /* Common */
  nh.param<int>("urimap/common/frequency", params.common.frequency, 10);

  /* Topics */
  nh.param<std::string>("urimap/topics/input/observations", params.topics.input.observations, "/cones/observed");
  nh.param<std::string>("urimap/topics/input/map", params.topics.input.map, "/map/accumulated");
  
  /* Preproc */
  nh.param<float>("urimap/preproc/cluster_dist", params.preproc.cluster_dist, 0.5);

  /* Matcher */
  nh.param<float>("urimap/matcher/reconstruct/cone_size_width", params.matcher.reconstruct.cone_size_width, 0.228);
  nh.param<float>("urimap/matcher/reconstruct/cone_size_height", params.matcher.reconstruct.cone_size_height, 0.325);
  nh.param<float>("urimap/matcher/reconstruct/safety_factor", params.matcher.reconstruct.safety_factor, 1.2);

  /* Tracker */
  nh.param<float>("urimap/preproc/clusterDist", params.preproc.clusterDist, 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "urimap");
  ros::NodeHandle nh;

  Params params;
  loadParams(params, nh);

  // Object instances
  Preproc &preproc = Preproc::getInstance(params.preproc);
  Matcher &matcher = Matcher::getInstance(params.matcher);
  Tracker &tracker = Tracker::getInstance(params.tracker);

  // Subscribers & Publisher
  ros::Subscriber subPau = nh.subscribe("/cones/observed", 1, &Preproc::obsCallback, &preproc);
  ros::Subscriber subMap = nh.subscribe("/map/observed", 1, &Matcher::mapCallback, &matcher);

  message_filters::Subscriber<nav_msgs::Odometry> state_carSub(nh, "/state/car", 100);

  // cameras detections
  message_filters::Subscriber<geometry_msgs::PoseArray> left_detectionsSub(nh, "/camera/left/detections", 100);
  message_filters::Subscriber<geometry_msgs::PoseArray> right_detectionsSub(nh, "/camera/right/detections", 100);

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
}