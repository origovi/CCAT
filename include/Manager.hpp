/**
 * @file Manager.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Manager module.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>
#include <ccat/ExtrinsicsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <omp.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "modules/Matcher/Matcher.hpp"
#include "modules/Merger/Merger.hpp"
#include "modules/Preproc/Preproc.hpp"
#include "modules/Tracker/Tracker.hpp"
#include "structures/Buffer.hpp"
#include "structures/Params.hpp"
#include "utils/Time.hpp"

/**
 * @brief The Manager module is what coordinates all different Modules.
 * It is responsible for:
 * - Data synchronization
 * - Dynamic reconfigure objects
 * - Pipeline calls
 */
class Manager {
 private:
  /* ------------------ Private Constructor And Destructor ------------------ */

  Manager();
  ~Manager();

  /* -------------------------------- Modules ------------------------------- */
  Preproc *preproc;
  Matcher *matcherL, *matcherR;
  Merger *merger;
  Tracker *tracker;

  /* -------------------------- Private Attributes -------------------------- */
  Buffer<geometry_msgs::PoseArray::ConstPtr> *buffLeftBBs_, *buffRightBBs_;
  Buffer<nav_msgs::Odometry::ConstPtr> *buffOdom_;
  as_msgs::ObservationArray::ConstPtr latestObs_;  // Buffer don't needed

  ros::Publisher conesPub_;
  Params::Manager params_;
  ros::CallbackQueue *calibQueue_;

  /* Last run params */
  nav_msgs::Odometry::ConstPtr lastRunOdom_;
  as_msgs::ObservationArray::ConstPtr lastRunObs_;
  geometry_msgs::PoseArray::ConstPtr lastRunLeftBBs_, lastRunRightBBs_;
  geometry_msgs::PoseArray::ConstPtr lastRunValidLeftBBs_, lastRunValidRightBBs_;

  enum Mode { NONE,
              L_ONLY,
              L_CAM,
              R_CAM,
              BOTH_CAMS } mode_;

  enum Update { OBS,
                ODOM,
                L_BBS,
                R_BBS };

  /* ---------------------------- Private Methods --------------------------- */
  /**
   * @brief Runs the pipeline with last synchronized data.
   */
  void run() const;

  /**
   * @brief Checks whether or not the latest data is synchronized, if so, runs
   * the pipeline with it.
   * 
   * @param update 
   */
  void runIfPossible(const Update &update);

  /**
   * @brief Enters a calibration loop with the same data, allows to call
   * dynamic reconfigure callbacks.
   */
  void calibLoop() const;

  /**
   * @brief Updates the working mode. LiDAR only, camera only, ...
   */
  void updateMode();
  template <typename BufferedType>
  bool buffHasValidData(const Buffer<BufferedType> &buff) const;

 public:
  /* --------------------------- Singleton Pattern -------------------------- */

  static Manager &getInstance();
  Manager(Manager const &) = delete;
  void operator=(Manager const &) = delete;

  void init(ros::NodeHandle *const nh, const Params &params,
            const ros::Publisher &conesPub,
            dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_left,
            dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_right,
            ros::CallbackQueue *const calibQueue);

  /* Callbacks */
  void leftBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs);
  void rightBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void obsCallback(const as_msgs::ObservationArray::ConstPtr &observations);
};

#endif  // MANAGER_HPP