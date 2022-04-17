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

void Manager::run(const nav_msgs::Odometry::ConstPtr &odom,
                  const geometry_msgs::PoseArray::ConstPtr &leftBBs,
                  const geometry_msgs::PoseArray::ConstPtr &rightBBs) const {
  Time::tick("main");
  preproc->run(latestObs_, odom, leftBBs, rightBBs);
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
  if (latestObs_ == nullptr or buffOdom_->size() < 5) return;
  if (buffRightBBs_->size() < 5 and buffLeftBBs_->size() < 5) return;
  //if (ros::Time::now() - lastRunStamp_ < ros::Duration(params_.minTimeBetweenPubs)) return;

  lastRunStamp_ = ros::Time::now();
  //run();
}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

Manager &Manager::getInstance() {
  static Manager manager;
  return manager;
}

void Manager::init(ros::NodeHandle *const nh, const Params &params, const ros::Publisher &conesPub) {
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