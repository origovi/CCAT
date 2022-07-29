/**
 * @file main.cpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief Main file of CCAT
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#include <as_msgs/ConeArray.h>
#include <ccat/ExtrinsicsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "Manager.hpp"
#include "structures/Params.hpp"
#include "utils/Visualization.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ccat");
  ros::NodeHandle *const nh = new ros::NodeHandle;

  Params params(*nh);
  Visualization::init(nh);

  Manager &manager = Manager::getInstance();

  // Create another callback queue only for the camera extrinsics dyn_cfg
  // This way we can call dyn_cfg callbacks while being inside a
  // (default)GlobalCallbackQueue callback.
  ros::CallbackQueue calibQueue;

  /* Subscribers */
  ros::Subscriber subObs = nh->subscribe(params.common.topics.input.observations, 10, &Manager::obsCallback, &manager);
  ros::Subscriber subOdom = nh->subscribe(params.common.topics.input.odom, 10, &Manager::odomCallback, &manager);
  ros::Subscriber subLeftBBs = nh->subscribe(params.common.topics.input.left_bbs, 10, &Manager::leftBBsCallback, &manager);
  ros::Subscriber subRightBBs = nh->subscribe(params.common.topics.input.right_bbs, 10, &Manager::rightBBsCallback, &manager);

  /* Publisher */
  ros::Publisher pubCones = nh->advertise<as_msgs::ConeArray>(params.common.topics.output.cones, 1);

  /* Dynamic Reconfigure of camera extrinsics*/
  // We need to declare two NHs because there must be only one dyn_rec::Server per NH
  ros::NodeHandle nh_cfg_left(*nh, "cfg_left_cam"), nh_cfg_right(*nh, "cfg_right_cam");
  nh_cfg_left.setCallbackQueue(&calibQueue);
  nh_cfg_right.setCallbackQueue(&calibQueue);
  dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> cfgSrv_extr_left(nh_cfg_left), cfgSrv_extr_right(nh_cfg_right);

  /* Manager initilization */
  manager.init(nh, params, pubCones, cfgSrv_extr_left, cfgSrv_extr_right, &calibQueue);

  // Enjoy
  ros::spin();
}