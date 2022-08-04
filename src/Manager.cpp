#include "Manager.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

Manager::Manager() {}
Manager::~Manager() {
  delete matcherL, matcherR;

  delete buffLeftBBs_;
  delete buffRightBBs_;
  delete buffOdom_;
}

void Manager::run() const {
  Time::tick("main");
#pragma omp parallel num_threads(2)
#pragma omp single
  {
    // std::cout << omp_get_num_threads() << " threads" << std::endl;
    preproc->run(lastRunObs_, lastRunOdom_.first, lastRunLeftBBs_, lastRunRightBBs_);
    tracker->accumulate(preproc->getData());
    calibQueue_->callAvailable();
#pragma omp task
    matcherL->run(tracker->getObservations(), preproc->getBBs(matcherL->which));
#pragma omp task
    matcherR->run(tracker->getObservations(), preproc->getBBs(matcherR->which));
#pragma omp taskwait
    merger->run(matcherL->getData(), matcherR->getData());
    tracker->run(merger->getData());
    std::cout << std::endl;
    //preproc->reset();
    if (tracker->hasData()) conesPub_.publish(tracker->getData());
  }
  Time::tock("main");

  if (params_.static_calib and lastRunLeftBBs_ != nullptr and lastRunRightBBs_ != nullptr)
    calibLoop();
}

void Manager::runIfPossible(const Update &update) {
  if (mode_ == NONE) return;

  std::pair<Eigen::Affine3d, ros::Time> odom;
  geometry_msgs::PoseArray::ConstPtr leftBBs, rightBBs;

  if (update == OBS) {
    odom = lastRunOdom_;
    leftBBs = lastRunLeftBBs_;
    rightBBs = lastRunRightBBs_;
  } else {
    if (mode_ == L_ONLY) {
      if (update != ODOM) return;
      ROS_WARN("L_ONLY");
      tf::poseMsgToEigen(buffOdom_->newestElem().first->pose.pose, odom.first);
      odom.second = buffOdom_->newestElem().second;
    } else if (mode_ == L_CAM) {
      if (update == R_BBS) return;
      nav_msgs::Odometry::ConstPtr o1, o2;
      if (!buffLeftBBs_->dataWithBoundsInBuffParam(*buffOdom_, leftBBs, o1, o2)) return;
      if (leftBBs->header.stamp < lastRunLeftStamp_) return;
      odom.first = poseInterpolation(leftBBs->header.stamp, o1, o2);
      odom.second = leftBBs->header.stamp;
      ROS_WARN("L_CAM");
    } else if (mode_ == R_CAM) {
      if (update == L_BBS) return;
      nav_msgs::Odometry::ConstPtr o1, o2;
      if (!buffRightBBs_->dataWithBoundsInBuffParam(*buffOdom_, rightBBs, o1, o2)) return;
      if (rightBBs->header.stamp < lastRunRightStamp_) return;
      odom.first = poseInterpolation(rightBBs->header.stamp, o1, o2);
      odom.second = rightBBs->header.stamp;
      ROS_WARN("R_CAM");
    }
    // BOTH_CAMS
    else {
      nav_msgs::Odometry::ConstPtr odomLeft1, odomLeft2, odomRight1, odomRight2;
      geometry_msgs::PoseArray::ConstPtr rightBBsTemp, leftBBsTemp;
      bool leftSync = buffLeftBBs_->dataWithBoundsInBuffParam(*buffOdom_, leftBBsTemp, odomLeft1, odomLeft2);
      bool rightSync = buffRightBBs_->dataWithBoundsInBuffParam(*buffOdom_, rightBBsTemp, odomRight1, odomRight2);
      if (!leftSync and !rightSync) return;
      if (leftSync and leftBBsTemp->header.stamp > lastRunLeftStamp_) {
        odom.first = poseInterpolation(leftBBsTemp->header.stamp, odomLeft1, odomLeft2);
        odom.second = leftBBsTemp->header.stamp;
        leftBBs = leftBBsTemp;
      }
      if (rightSync and rightBBsTemp->header.stamp > lastRunRightStamp_) {
        if (leftSync and leftBBsTemp->header.stamp > lastRunLeftStamp_) {
          if (rightBBsTemp->header.stamp < leftBBsTemp->header.stamp) {
            leftBBs = nullptr;
            odom.first = poseInterpolation(rightBBsTemp->header.stamp, odomRight1, odomRight2);
            odom.second = rightBBsTemp->header.stamp;
            rightBBs = rightBBsTemp;
          } else if (rightBBsTemp->header.stamp == leftBBsTemp->header.stamp) {
            rightBBs = rightBBsTemp;
          }
        } else {
          odom.first = poseInterpolation(rightBBsTemp->header.stamp, odomRight1, odomRight2);
          odom.second = rightBBsTemp->header.stamp;
          rightBBs = rightBBsTemp;
        }
      }
      ROS_WARN("BOTH_CAMS");
    }
    if (odom.second < lastRunOdom_.second) {
      ROS_WARN("EP");
      std::cout << odom.second << std::endl;
      return;
    }
  }

  lastRunObs_ = latestObs_;
  lastRunOdom_ = odom;
  lastRunRightBBs_ = rightBBs;
  lastRunLeftBBs_ = leftBBs;
  if (rightBBs != nullptr) lastRunRightStamp_ = rightBBs->header.stamp;
  if (leftBBs != nullptr) lastRunLeftStamp_ = leftBBs->header.stamp;

  // Run
  run();
}

void Manager::updateMode() {
  mode_ = NONE;
  if (latestObs_ != nullptr) {
    if (buffHasValidData(*buffOdom_)) {
      mode_ = L_ONLY;
      if (!params_.only_lidar) {
        if (buffHasValidData(*buffLeftBBs_)) {
          mode_ = L_CAM;
          if (buffHasValidData(*buffRightBBs_)) {
            mode_ = BOTH_CAMS;
          }
        } else if (buffHasValidData(*buffLeftBBs_)) {
          mode_ = R_CAM;
        }
      }
    }
  }
  // Update Cone's class parameter (to not invalidate cones when cameras are missing)
  Cone::bothCams = mode_ == Mode::BOTH_CAMS;
}

void Manager::calibLoop() const {
  ros::WallRate calibRate(5);
  while (ros::ok()) {
    calibQueue_->callAvailable();
    matcherL->run(tracker->getObservations(), preproc->getBBs(matcherL->which));
    matcherR->run(tracker->getObservations(), preproc->getBBs(matcherR->which));
    calibRate.sleep();
  }
}

template <typename BufferedType>
bool Manager::buffHasValidData(const Buffer<BufferedType> &buff) const {
  return buff.size() >= 5 and ros::Time::now() - buff.newestStamp() < ros::Duration(1.0);
}

Eigen::Affine3d Manager::poseInterpolation(const ros::Time &t, const nav_msgs::Odometry::ConstPtr &o1, const nav_msgs::Odometry::ConstPtr &o2) const {
  ros::Time t1 = o1->header.stamp + ros::Duration(timeDiff_);
  ros::Time t2 = o2->header.stamp + ros::Duration(timeDiff_);

  double alpha = 0.0;
  if (t2 != t1) {
    alpha = (t - t1).toSec() / (t2 - t1).toSec();
  }

  Eigen::Affine3d a1, a2;
  tf::poseMsgToEigen(o1->pose.pose, a1);
  tf::poseMsgToEigen(o2->pose.pose, a2);
  Eigen::Quaternion<double> rot1(a1.linear());
  Eigen::Quaternion<double> rot2(a2.linear());

  Eigen::Affine3d res;
  res.translation() = (1.0 - alpha) * a1.translation() + alpha * a2.translation();
  res.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();
  return res;
}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

Manager &Manager::getInstance() {
  static Manager manager;
  return manager;
}

void Manager::init(ros::NodeHandle *const nh, const Params &params,
                   const ros::Publisher &conesPub,
                   dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_left,
                   dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_right,
                   ros::CallbackQueue *const calibQueue,
                   dynamic_reconfigure::Server<ccat::TimeDiffConfig> &cfgSrv_timeDiff) {
  conesPub_ = conesPub;
  params_ = params.manager;
  calibQueue_ = calibQueue;

  /* Initialize modules */
  preproc = &Preproc::getInstance();
  preproc->init(params.preproc);
  matcherL = new Matcher(params.matcherL, nh, cfgSrv_extr_left, Matcher::LEFT);
  matcherR = new Matcher(params.matcherR, nh, cfgSrv_extr_right, Matcher::RIGHT);
  merger = &Merger::getInstance();
  merger->init(params.merger);
  tracker = &Tracker::getInstance();
  tracker->init(params.tracker);

  cfgSrv_extr_left.setCallback(boost::bind(&Matcher::cfgCallback, matcherL, _1, _2));
  cfgSrv_extr_right.setCallback(boost::bind(&Matcher::cfgCallback, matcherR, _1, _2));

  cfgSrv_timeDiff.setCallback(boost::bind(&Manager::cfgCallback, this, _1, _2));

  /* Initialize attributes */
  buffLeftBBs_ = new Buffer<geometry_msgs::PoseArray::ConstPtr>(params.manager.bufferTempMem);
  buffRightBBs_ = new Buffer<geometry_msgs::PoseArray::ConstPtr>(params.manager.bufferTempMem);
  buffOdom_ = new Buffer<nav_msgs::Odometry::ConstPtr>(params.manager.bufferTempMem);
}

/* -------------------------------- Callbacks ------------------------------- */

void Manager::leftBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs) {
  buffLeftBBs_->add(bbs, bbs->header.stamp);
  updateMode();
  runIfPossible(L_BBS);
}

void Manager::rightBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs) {
  buffRightBBs_->add(bbs, bbs->header.stamp);
  updateMode();
  runIfPossible(R_BBS);
}

void Manager::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  buffOdom_->add(odom, odom->header.stamp + ros::Duration(timeDiff_));
  updateMode();
  runIfPossible(ODOM);
}

void Manager::obsCallback(const as_msgs::ObservationArray::ConstPtr &observations) {
  latestObs_ = observations;
  updateMode();
  runIfPossible(OBS);
}

void Manager::cfgCallback(const ccat::TimeDiffConfig &config, uint32_t level) {
  timeDiff_ = config.time_diff;
  ROS_WARN("cfg callback");
}