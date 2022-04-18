#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>
#include <ccat/ExtrinsicsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "modules/Matcher/Matcher.hpp"
#include "modules/Merger/Merger.hpp"
#include "modules/Preproc/Preproc.hpp"
#include "modules/Tracker/Tracker.hpp"
#include "structures/Buffer.hpp"
#include "structures/Params.hpp"
#include "utils/Time.hpp"

class Manager {
 private:
  /* ---------------------- Constructor And Destructor ---------------------- */
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

  /* Last run params */
  nav_msgs::Odometry::ConstPtr lastRunOdom_;
  as_msgs::ObservationArray::ConstPtr lastRunObs_;
  geometry_msgs::PoseArray::ConstPtr lastRunLeftBBs_, lastRunRightBBs_;

  /* ---------------------------- Private Methods --------------------------- */
  void run() const;
  void runIfPossible();
  template <typename BufferedType>
  bool buffHasValidData(const Buffer<BufferedType> &buff) const;

 public:
  static Manager &getInstance();
  Manager(Manager const &) = delete;
  void operator=(Manager const &) = delete;

  void init(ros::NodeHandle *const nh, const Params &params,
            const ros::Publisher &conesPub,
            dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_left,
            dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_right);

  /* Callbacks */
  void leftBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs);
  void rightBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void obsCallback(const as_msgs::ObservationArray::ConstPtr &observations);
};

#endif  // MANAGER_HPP