#include "Preproc.hpp"

/**
 * CONSTRUCTORS
 */
Preproc::Preproc() : hasData_(false) {}

/**
 * DESTRUCTORS
 */
Preproc::~Preproc() {}

/**
 * PRIVATE METHODS
 */

std::list<const Observation *> Preproc::possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const {
  std::list<const Observation *> res;
  res.push_back(&allObs[pointIndex]);
  std::vector<size_t> ar(observationsKDT.neighborhood_indices(allObs[pointIndex].centroid, params_.cluster_dist));
  for (const size_t &ind : ar) {
    if (!visited[ind]) {
      visited[ind] = true;
      res.splice(res.end(), possiblesSamePoint(ind, observationsKDT, allObs, visited));
    }
  }
  return res;
}

std::vector<Observation> Preproc::preprocess(const as_msgs::ObservationArray &observations) const {
  std::vector<Observation> allObs(cvrs::as_obsVec2ObsVec(observations.observations));
  KDTree observationsKDT(cvrs::obsVec2PointVec(allObs));
  std::vector<bool> visited(observations.observations.size(), false);
  std::vector<Observation> res;
  res.reserve(allObs.size());

  for (size_t i = 0; i < observations.observations.size(); ++i) {
    if (!visited[i]) {
      res.emplace_back(possiblesSamePoint(i, observationsKDT, allObs, visited));
    }
  }
  return res;
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

/* Callbacks */
void Preproc::obsCallback(const as_msgs::ObservationArray &newObservations) {
  if (!newObservations.observations.empty()) {
    currentObservations_ = preprocess(newObservations);
    hasData_ = true;
    //KDTree *newObservationsKDT = new KDTree(preprocess(newObservations));
    //delete observationsTree_;
    //observationsTree_ = new KDTree(observationsToPointVec(newObservations.observations));
  }
}

/* Getters */
const std::vector<Observation> &Preproc::getCurrentObservations() const {
  return currentObservations_;
}

const bool &Preproc::hasData() const {
  return hasData_;
}