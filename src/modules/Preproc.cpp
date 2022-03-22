#include "modules/Preproc.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */
Preproc::Preproc() : hasData_(false) {}
Preproc::~Preproc() {}

/**
 * PRIVATE METHODS
 */

std::list<const Observation*> Preproc::possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const {
  std::list<const Observation*> res;
  res.push_back(&allObs[pointIndex]);
  std::vector<size_t> ar(observationsKDT.neighborhood_indices(allObs[pointIndex].centroid_global, params_.cluster_dist));
  for (const size_t &ind : ar) {
    if (!visited[ind]) {
      visited[ind] = true;
      res.splice(res.end(), possiblesSamePoint(ind, observationsKDT, allObs, visited));
    }
  }
  return res;
}

void Preproc::preprocess(const as_msgs::ObservationArray &observations, const Eigen::Affine3d &carTf) {
  std::vector<Observation> allObs;
  cvrs::as_obsVec2ObsVec(observations.observations, allObs);
  std::vector<Point> allCentroids;
  cvrs::obsVec2PointVec(allObs, allCentroids);
  KDTree observationsKDT(allCentroids);
  std::vector<bool> visited(observations.observations.size(), false);
  std::vector<Observation> res;
  res.reserve(allObs.size());

  // Cluster the observations
  for (size_t i = 0; i < observations.observations.size(); ++i) {
    if (!visited[i]) {
      res.emplace_back(possiblesSamePoint(i, observationsKDT, allObs, visited));
    }
  }
  res.shrink_to_fit();

  // Transform the points to car location
  for (size_t i = 0; i < res.size(); ++i) {
    pcl::transformPointCloud(*res[i].pcl, *res[i].pcl, carTf);
    res[i].centroid_base_link = res[i].centroid_global.transformed(carTf);
    res[i].distToCar = Point::dist(res[i].centroid_base_link);
  }
  currentObservations_ = res;
}

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */

Preproc &Preproc::getInstance() {
  static Preproc preproc;
  return preproc;
}

/* Init */

void Preproc::init(ros::NodeHandle *const &nh, const Params::Preproc &params) {
  nh_ = nh;
  params_ = params;
}

void Preproc::reset() {
  hasData_ = false;
}

/* Callbacks */

void Preproc::callback(const as_msgs::ObservationArray::ConstPtr &newObservations,
                       const nav_msgs::Odometry::ConstPtr &carPos,
                       const geometry_msgs::PoseArray::ConstPtr &leftDetections,
                       const geometry_msgs::PoseArray::ConstPtr &rightDetections) {
  ROS_INFO("CALLBACK");
  Eigen::Affine3d carTf;
  tf::poseMsgToEigen(carPos->pose.pose, carTf);
  carTf = carTf.inverse();
  // Invert y and z axis
  // extrinsics_car_.translation().z() *= -1;
  // extrinsics_car_.translation().y() *= -1;

  leftBbs_ = leftDetections;
  rightBbs_ = rightDetections;

  if (newObservations->observations.empty())
    ROS_WARN("Reading empty observations");

  preprocess(*newObservations, carTf);
  std::cout << "orig size: " << newObservations->observations.size() << " process size: " << currentObservations_.size() << std::endl;
  hasData_ = true;
}

/* Getters */

Matcher::RqdData Preproc::getData(const Matcher::Which &which) const {
  Matcher::RqdData res;
  res.observations = currentObservations_;

  // For each matcher, we will return the correspondant BB
  switch (which) {
    case Matcher::Which::LEFT:
      res.bbs = leftBbs_;
      break;

    case Matcher::Which::RIGHT:
      res.bbs = rightBbs_;
      break;
  }
  return res;
}

const bool &Preproc::hasData() const {
  return hasData_;
}