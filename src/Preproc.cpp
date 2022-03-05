#include "Preproc.hpp"

/**
 * CONSTRUCTORS
 */
Preproc::Preproc(const Params::Preproc &params) : params_(params), hasData_(false) {}

/**
 * DESTRUCTORS
 */
Preproc::~Preproc() {}

/**
 * PRIVATE METHODS
 */

std::list<size_t> Preproc::possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const {
  std::list<size_t> res;
  res.push_back(pointIndex);
  std::vector<size_t> ar(observationsKDT.neighborhood_indices(allObs[pointIndex].p, params_.cluster_dist));
  for (const size_t &ind : ar) {
    if (!visited[ind]) {
      visited[ind] = true;
      res.splice(res.end(), possiblesSamePoint(ind, observationsKDT, allObs, visited));
    }
  }
  return res;
}

Observation Preproc::centroidObs(const std::list<size_t> &points, const std::vector<Observation> &allObs) const {
  Observation res;
  for (size_t i = 0; i < points.size(); ++i) {
    res += allObs[i];
  }
  return res /= int(points.size());
}

std::vector<Observation> Preproc::preprocess(const as_msgs::ObservationArray &observations) const {
  std::vector<Observation> allObs(cvrs::as_obsVec2ObsVec(observations.observations));
  KDTree observationsKDT(cvrs::obsVec2PointVec(allObs));
  std::vector<bool> visited(observations.observations.size(), false);
  std::vector<Observation> res;
  res.reserve(allObs.size());

  for (size_t i = 0; i < observations.observations.size(); ++i) {
    if (!visited[i]) {
      res.push_back(centroidObs(possiblesSamePoint(i, observationsKDT, allObs, visited), allObs));
    }
  }
  return res;
}

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */
Preproc &Preproc::getInstance(const Params::Preproc &params) {
  static Preproc preproc(params);
  return preproc;
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