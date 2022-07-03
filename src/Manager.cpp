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
    preproc->run(lastRunObs_, lastRunOdom_, lastRunLeftBBs_, lastRunRightBBs_);
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

  nav_msgs::Odometry::ConstPtr odom;
  geometry_msgs::PoseArray::ConstPtr leftBBs, rightBBs;

  if (update == OBS) {
    odom = lastRunOdom_;
    leftBBs = lastRunLeftBBs_;
    rightBBs = lastRunRightBBs_;
  } else {
    if (mode_ == L_ONLY) {
      if (update != ODOM) return;
      ROS_WARN("L_ONLY");
      odom = buffOdom_->newestElem().first;
    } else if (mode_ == L_CAM) {
      if (update == R_BBS) return;
      if (!buffOdom_->isSynchWith(*buffLeftBBs_, odom, leftBBs)) return;
      if (lastRunOdom_ != nullptr and lastRunOdom_->header.stamp > odom->header.stamp) return;
      if (odom == lastRunOdom_ and leftBBs == lastRunLeftBBs_) return;
      ROS_WARN("L_CAM");
    } else if (mode_ == R_CAM) {
      if (update == L_BBS) return;
      if (!buffOdom_->isSynchWith(*buffRightBBs_, odom, rightBBs)) return;
      if (lastRunOdom_ != nullptr and lastRunOdom_->header.stamp > odom->header.stamp) return;
      if (odom == lastRunOdom_ and rightBBs == lastRunRightBBs_) return;
      ROS_WARN("R_CAM");
    }
    // BOTH_CAMS
    else {
      nav_msgs::Odometry::ConstPtr odomLeft, odomRight;
      geometry_msgs::PoseArray::ConstPtr rightBBsTemp, leftBBsTemp;
      bool rightSync = buffOdom_->isSynchWith(*buffRightBBs_, odomRight, rightBBsTemp);
      bool leftSync = buffOdom_->isSynchWith(*buffLeftBBs_, odomLeft, leftBBsTemp);
      if (!leftSync and !rightSync) return;
      ROS_WARN("BOTH_CAMS");
      if (leftSync and rightSync) {
        if (odomRight == odomLeft) {
          odom = odomRight;
          leftBBs = leftBBsTemp;
          rightBBs = rightBBsTemp;
        } else if (odomRight->header.stamp > odomLeft->header.stamp) {
          if (lastRunOdom_ != nullptr and odomLeft->header.stamp > lastRunOdom_->header.stamp) {
            odom = odomLeft;
            leftBBs = leftBBsTemp;
          } else {
            odom = odomRight;
            rightBBs = rightBBsTemp;
          }
        } else {
          if (lastRunOdom_ != nullptr and odomRight->header.stamp > lastRunOdom_->header.stamp) {
            odom = odomRight;
            rightBBs = rightBBsTemp;
          } else {
            odom = odomLeft;
            leftBBs = leftBBsTemp;
          }
        }
      } else if (leftSync) {
        odom = odomLeft;
        leftBBs = leftBBsTemp;
      } else {
        odom = odomRight;
        rightBBs = rightBBsTemp;
      }
      if (lastRunOdom_ != nullptr and lastRunOdom_->header.stamp > odom->header.stamp) return;
    }
  }

  lastRunObs_ = latestObs_;
  lastRunOdom_ = odom;
  lastRunRightBBs_ = rightBBs;
  lastRunLeftBBs_ = leftBBs;

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
                   ros::CallbackQueue *const calibQueue) {
  conesPub_ = conesPub;
  params_ = params.manager;
  calibQueue_ = calibQueue;

  /* Initialize modules */
  preproc = &Preproc::getInstance();
  preproc->init(params.preproc);
  matcherL = new Matcher(params.matcherL, nh, Matcher::LEFT);
  matcherR = new Matcher(params.matcherR, nh, Matcher::RIGHT);
  merger = &Merger::getInstance();
  merger->init(params.merger);
  tracker = &Tracker::getInstance();
  tracker->init(params.tracker);

  cfgSrv_extr_left.setCallback(boost::bind(&Matcher::cfgCallback, matcherL, _1, _2));
  cfgSrv_extr_right.setCallback(boost::bind(&Matcher::cfgCallback, matcherR, _1, _2));

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
  buffOdom_->add(odom, odom->header.stamp);
  updateMode();
  runIfPossible(ODOM);
}

void Manager::obsCallback(const as_msgs::ObservationArray::ConstPtr &observations) {
  latestObs_ = observations;
  updateMode();
  runIfPossible(OBS);
}