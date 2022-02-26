#include <ros/ros.h>

#include "Tracker.hpp"
#include "Matcher.hpp"

void loadParams(Params &params, const ros::NodeHandle &nh) {
  nh.param<float>("urimap/tracker/clusterDist", params.tracker.clusterDist, 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "urimap");
  ros::NodeHandle nh;

  Params params;
  loadParams(params, nh);

  // Object instances
  Tracker &tracker = Tracker::getInstance(params.tracker);
  Matcher &matcher = Matcher::getInstance(params.matcher);

  // Publisher & Subscriber
  ros::Subscriber subPau = nh.subscribe("/cones/observed", 1, &Tracker::mapCallback, &tracker);
  
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (tracker.hasData()) {
      
    }
    rate.sleep();
  }
}