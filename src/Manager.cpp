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
  preproc->run(lastRunObs_, lastRunOdom_, lastRunLeftBBs_, lastRunRightBBs_);
  tracker->accumulate(preproc->getData());
  matcherL->run(tracker->getObservations(), preproc->getBBs(matcherL->which));
  matcherR->run(tracker->getObservations(), preproc->getBBs(matcherR->which));
  merger->run(matcherL->getData(), matcherR->getData());
  tracker->run(merger->getData());
  std::cout << std::endl;
  Time::tock("main");
  //preproc->reset();
  if (tracker->hasData()) conesPub_.publish(tracker->getData());
}

void Manager::runIfPossible() {
  // If some of the data is missing or we haven't got enough, return
  if (latestObs_ == nullptr or !buffHasValidData(*buffOdom_)) return;
  nav_msgs::Odometry::ConstPtr odom = buffOdom_->newestElem();
  geometry_msgs::PoseArray::ConstPtr leftBBs, rightBBs;

  if (buffHasValidData(*buffRightBBs_) and buffRightBBs_->isSynchWith(*buffOdom_)) {
    rightBBs = buffRightBBs_->newestElem();
  }
  if (buffHasValidData(*buffLeftBBs_) and buffLeftBBs_->isSynchWith(*buffOdom_)) {
    leftBBs = buffLeftBBs_->newestElem();
  }

  // Don't run same BBs with same Observations and Odom twice
  if (odom == lastRunOdom_ and latestObs_ == lastRunObs_) {
    if (lastRunLeftBBs_ == leftBBs) {
      leftBBs = nullptr;
    }
    if (lastRunRightBBs_ == rightBBs) {
      rightBBs = nullptr;
    }
    // No need to re-run:
    // 1. odom and observations are same &&
    // 2. BBs are either the same or not in synch
    if (rightBBs == nullptr and leftBBs == nullptr) return;
  }

  // Update latest vars
  lastRunOdom_ = odom;
  lastRunObs_ = latestObs_;
  lastRunLeftBBs_ = leftBBs;
  lastRunRightBBs_ = rightBBs;

  // Run
  run();
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
                   dynamic_reconfigure::Server<ccat::ExtrinsicsConfig> &cfgSrv_extr_right) {
  conesPub_ = conesPub;
  params_ = params.manager;

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
  runIfPossible();
}

void Manager::rightBBsCallback(const geometry_msgs::PoseArray::ConstPtr &bbs) {
  buffRightBBs_->add(bbs, bbs->header.stamp);
  runIfPossible();
}
void Manager::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  buffOdom_->add(odom, odom->header.stamp);
  runIfPossible();
}
void Manager::obsCallback(const as_msgs::ObservationArray::ConstPtr &observations) {
  latestObs_ = observations;
  runIfPossible();
}