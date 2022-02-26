#include "Tracker.hpp"

/**
 * CONSTRUCTORS
 */
Tracker::Tracker(const Params::Tracker &params) : params_(params), hasData_(false) {}

/**
 * DESTRUCTORS
 */
Tracker::~Tracker() {}

/**
 * PRIVATE METHODS
 */

std::list<size_t> Tracker::possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Point> &allPoints, std::vector<bool> &visited) const {
  std::list<size_t> res;
  res.push_back(pointIndex);
  std::vector<size_t> ar(observationsKDT.neighborhood_indices(allPoints[pointIndex], params_.clusterDist));
  for (const size_t &ind : ar) {
    if (!visited[ind]) {
      visited[ind] = true;
      res.splice(res.end(), possiblesSamePoint(ind, observationsKDT, allPoints, visited));
    }
  }
  return res;
}

Point Tracker::centroidPoint(const std::list<size_t> &points, const std::vector<Point> &allPoints) const {
  Point res;
  for (size_t i = 0; i < points.size(); ++i) {
    res += allPoints[i];
  }
  return res /= int(points.size());
}

std::list<Point> Tracker::preprocess(const as_msgs::ObservationArray &observations) const {
  std::vector<Point> allPoints(cvrs::observationsToPointVec(observations.observations));
  KDTree observationsKDT(allPoints);
  std::vector<bool> visited(observations.observations.size(), false);
  std::list<Point> res;

  for (size_t i = 0; i < observations.observations.size(); ++i) {
    if (!visited[i]) {
      res.push_back(centroidPoint(possiblesSamePoint(i, observationsKDT, allPoints, visited), allPoints));
    }
  }
  return res;
}

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */
Tracker &Tracker::getInstance(const Params::Tracker &params) {
  static Tracker tracker(params);
  return tracker;
}

/* Callbacks */
void Tracker::mapCallback(const as_msgs::ObservationArray &newObservations) {
  if (!newObservations.observations.empty()) {
    delete observationsTree_;
    //observationsTree_ = new KDTree(observationsToPointVec(newObservations.observations));
  }
}

/* Getters */
const std::vector<as_msgs::Observation> &Tracker::getCurrentObservations() const {
  return currentObservations_;
}

const bool &Tracker::hasData() const {
  return hasData_;
}