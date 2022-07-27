#include "modules/Preproc/Preproc.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */
Preproc::Preproc() : hasData_(false) {}

/**
 * PRIVATE METHODS
 */

std::list<const Observation *> Preproc::possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const {
  std::list<const Observation *> res;
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

void Preproc::preprocess(const as_msgs::ObservationArray &observations) {
  // Convert as_msgs::Observation to Observation
  std::vector<Observation> allObs;
  allObs.reserve(observations.observations.size());
  for (const as_msgs::Observation &obs : observations.observations) {
    allObs.emplace_back(obs);
  }

  // Publish markers
  if (params_.publish_markers) vis_.publishInputMarkers(allObs);
  
  // Create a centroid vector to build a KDTree
  std::vector<Point> allCentroids;
  allCentroids.reserve(allObs.size());
  for (const Observation &obs : allObs) {
    allCentroids.push_back(obs.centroid_global);
  }
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

void Preproc::init(const Params::Preproc &params) {
  params_ = params;
  vis_.init(params);
}

void Preproc::reset() {
  hasData_ = false;
}

/* Callbacks */

void Preproc::run(const as_msgs::ObservationArray::ConstPtr &newObservations,
                  const Eigen::Affine3d &odom,
                  const geometry_msgs::PoseArray::ConstPtr &leftDetections,
                  const geometry_msgs::PoseArray::ConstPtr &rightDetections) {
  currentObservations_.clear();
  // if (odom == nullptr) {
  //   ROS_ERROR("Odom is nullptr, aborting");
  //   return;
  // }
  //tf::poseMsgToEigen(odom->pose.pose, carTf_);

  // Invert y and z axis
  static const Eigen::Matrix4d aux = (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0,
                                      0.0, -1.0, 0.0, 0.0,
                                      0.0, 0.0, -1.0, 0.0,
                                      0.0, 0.0, 0.0, 1.0)
                                         .finished();
  // carTf_.translation().y() *= -1;
  // carTf_.translation().z() *= -1;
  carTf_ = odom.inverse();
  carTf_.matrix() = carTf_.matrix() * aux;

  leftBBs_ = leftDetections;
  rightBBs_ = rightDetections;

  if (newObservations == nullptr)
    return;
  if (newObservations->observations.empty()) {
    ROS_WARN("Reading empty observations");
    return;
  }

  // Only preprocess observations if they are different from the lasts
  if (newObservations->header.stamp != currentObservationsStamp_) {
    preprocess(*newObservations);
    currentObservationsStamp_ = newObservations->header.stamp;
  }
  // std::cout << "orig size: " << newObservations->observations.size() << " process size: " << currentObservations_.size() << std::endl;
  hasData_ = true;
}

/* Getters */

const geometry_msgs::PoseArray::ConstPtr &Preproc::getBBs(const Matcher::Which &which) const {
  // For each matcher, we will return the correspondant BB
  switch (which) {
    case Matcher::Which::LEFT:
      return leftBBs_;

    default:
      return rightBBs_;
  }
}

std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> Preproc::getData() const {
  return {currentObservations_, carTf_};
}

const bool &Preproc::hasData() const {
  return hasData_;
}

const Eigen::Affine3d &Preproc::getCarTf() const {
  return carTf_;
}