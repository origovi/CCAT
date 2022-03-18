#include <geometry_msgs/PoseArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <ccat/ExtrinsicsConfig.h>

#include "structures/Params.hpp"
#include "modules/Matcher.hpp"
#include "modules/Preproc.hpp"
#include "modules/Tracker.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ccat");
  ros::NodeHandle *const nh(new ros::NodeHandle("ccat"));

  Params params(*nh);

  /* Object instances */

  Preproc &preproc = Preproc::getInstance();
  preproc.init(nh, params.preproc);
  Matcher matcherL(params.matcherL, nh, Matcher::LEFT);
  Matcher matcherR(params.matcherR, nh, Matcher::RIGHT);
  //Merger &merger = Merger::getInstance();
  Tracker &tracker = Tracker::getInstance();
  tracker.init(nh, params.tracker);

  /* Dynamic Reconfigure of camera extrinsics*/

  // We need to declare two NHs because there must be only one dyn_rec::Server per NH
  ros::NodeHandle nh_cfg_left(*nh, "cfg_left_cam"), nh_cfg_right(*nh, "cfg_right_cam");
  dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> cfgServer_extrinsics_left(nh_cfg_left), cfgServer_extrinsics_right(nh_cfg_right);
  cfgServer_extrinsics_left.setCallback(boost::bind(&Matcher::cfgCallback, &matcherL, _1, _2));
  cfgServer_extrinsics_right.setCallback(boost::bind(&Matcher::cfgCallback, &matcherR, _1, _2));

  /* Subscribers */

  message_filters::Subscriber<as_msgs::ObservationArray> obsSub(*nh, params.common.topics.input.observations, 100);
  message_filters::Subscriber<nav_msgs::Odometry> state_carSub(*nh, params.common.topics.input.odometry, 100);
  message_filters::Subscriber<geometry_msgs::PoseArray> left_bbSub(*nh, params.common.topics.input.left_bbs, 100);
  message_filters::Subscriber<geometry_msgs::PoseArray> right_bbSub(*nh, params.common.topics.input.right_bbs, 100);
  typedef message_filters::sync_policies::ApproximateTime<as_msgs::ObservationArray, nav_msgs::Odometry, geometry_msgs::PoseArray, geometry_msgs::PoseArray> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> syncDetections(MySyncPolicy(100), obsSub, state_carSub, left_bbSub, right_bbSub);
  syncDetections.registerCallback(boost::bind(&Preproc::callback, &preproc, _1, _2, _3, _4));

  ros::Rate rate(params.common.frequency);
  while (ros::ok()) {
    ros::spinOnce();
    if (preproc.hasData()) {
      matcherL.run(preproc.getData(matcherL.which));
      matcherR.run(preproc.getData(matcherR.which));
      //merger.run(matcherL.getData(), matcherR.getData());
      //tracker.run(merger.getData());
    }
    rate.sleep();
  }
  delete nh;
}