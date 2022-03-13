#include <geometry_msgs/PoseArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "structures/Params.hpp"
#include "modules/Matcher.hpp"
#include "modules/Preproc.hpp"
#include "modules/Tracker.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "/AS/P/ccat");
  ros::NodeHandle *const nh(new ros::NodeHandle);

  Params params(*nh);

  // Object instances
  Preproc &preproc = Preproc::getInstance();
  preproc.init(nh, params.preproc);
  Matcher &matcher = Matcher::getInstance();
  matcher.init(nh, params.matcher);
  Tracker &tracker = Tracker::getInstance();
  tracker.init(nh, params.tracker);

  // Subscribers & Publisher
  ros::Subscriber subPau = nh->subscribe(params.common.topics.input.observations, 1, &Preproc::obsCallback, &preproc);
  // ros::Subscriber subMap = nh->subscribe("/map/observed", 1, &Matcher::mapCallback, &matcher);

  message_filters::Subscriber<nav_msgs::Odometry> state_carSub(*nh, params.common.topics.input.odometry, 100);

  // cameras detections
  message_filters::Subscriber<geometry_msgs::PoseArray> left_detectionsSub(*nh, params.matcher.topics.input.left_detections, 100);
  message_filters::Subscriber<geometry_msgs::PoseArray> right_detectionsSub(*nh, params.matcher.topics.input.right_detections, 100);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseArray, geometry_msgs::PoseArray> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> syncDetections(MySyncPolicy(100), state_carSub, left_detectionsSub, right_detectionsSub);
  syncDetections.registerCallback(boost::bind(&Matcher::locationAndBbsCbk, &matcher, _1, _2, _3));

  ros::Rate rate(params.common.frequency);
  while (ros::ok()) {
    ros::spinOnce();
    if (preproc.hasData() and matcher.hasData()) {
      matcher.run(preproc.getCurrentObservations());
      //tracker.run(matcher.getCurrentCones());
    }
    rate.sleep();
  }
  delete nh;
}