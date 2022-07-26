#include "modules/Tracker/Tracker.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */

Tracker::Tracker() {
  nextId_ = 0;
}

/**
 * PRIVATE METHODS
 */

void Tracker::getTrackingPoints(std::vector<Point> &points, std::vector<Cone *> &trackingPtrs) {
  points.resize(cones_.size());
  trackingPtrs.resize(cones_.size());
  size_t ind = 0;
  for (std::map<size_t, Cone>::iterator it = cones_.begin(); it != cones_.end(); it++) {
    points[ind] = it->second.position_global();
    trackingPtrs[ind] = &(it->second);
    ind++;
  }
}

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */

Tracker &Tracker::getInstance() {
  static Tracker matcher;
  return matcher;
}

/* Init */

void Tracker::init(const Params::Tracker &params) {
  params_ = params;
  vis_.init(params);
  Cone::params = params_.cone;
}

void Tracker::accumulate(const std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> &data) {
  size_t oldNumCones = cones_.size();
  if (cones_.empty()) {
    for (const Observation &obs : data.first) {
      cones_.emplace_hint(cones_.end(), std::piecewise_construct, std::forward_as_tuple(nextId_), std::forward_as_tuple(obs, nextId_));
      nextId_++;
    }
  } else {
    std::vector<Point> points(cones_.size());
    std::vector<Cone *> trackingPtrs(cones_.size());
    getTrackingPoints(points, trackingPtrs);
    KDTree tucutu(points);
    for (const Observation &obs : data.first) {
      const Point &pointToSearch = obs.centroid_global;
      size_t nearestInd = *tucutu.nearest_index(pointToSearch);
      double distSqToOldPos = Point::distSq(points[nearestInd], pointToSearch);
      // We have observed a known cone
      if (distSqToOldPos <= params_.same_cone_max_distSq) {
        trackingPtrs[nearestInd]->updateObs(obs, distSqToOldPos);
      }
      // We have observed a new cone
      else {
        cones_.emplace_hint(cones_.end(), std::piecewise_construct, std::forward_as_tuple(nextId_), std::forward_as_tuple(obs, nextId_));
        nextId_++;
      }
    }
  }

  // Update actual number of cones
  actualConeNumber_ = cones_.size() - oldNumCones;

  // Update the Observations to local space
  for (std::map<size_t, Cone>::iterator it = cones_.begin(); it != cones_.end(); it++) {
    it->second.updateLocal(data.second);
  }
}

void Tracker::run(const std::vector<ConeUpdate> &coneUpdates) {
  // Update all the cones
  for (const ConeUpdate &coneUpdate : coneUpdates) {
    std::map<size_t, Cone>::iterator it = cones_.find(coneUpdate.id);
    if (it != cones_.end()) {
      it->second.update(coneUpdate);
    }
    else {
      ROS_ERROR("Invalid ConeUpdate ID");
    }
  }

  if (params_.debug) {
    vis_.publishMergedMarkers(coneUpdates, cones_);
  }

  // Update the current cones (as_msgs)
  currentCones_.stamp = ros::Time::now();
  currentCones_.cones.clear();
  currentCones_.cones.reserve(cones_.size());
  for (std::map<size_t, Cone>::iterator it = cones_.begin(); it != cones_.end(); it++) {
    if (it->second.valid()) currentCones_.cones.push_back(it->second.getASCone());
  }
}

/* Callbacks */

/* Getters */

std::vector<Observation::Ptr> Tracker::getObservations() const {
  std::vector<Observation::Ptr> res;
  res.reserve(cones_.size());

  for (std::map<size_t, Cone>::const_iterator it = cones_.begin(); it != cones_.end(); it++) {
    res.push_back(it->second.obs);
  }
  return res;
}

const size_t &Tracker::getActualNumCones() const {
  return actualConeNumber_;
}

size_t Tracker::getTotalNumCones() const {
  return cones_.size();
}

const as_msgs::ConeArray &Tracker::getData() {
  // Publish markers, if debug
  if (params_.debug) {
    vis_.publishFinalMarkers(currentCones_);
  }
  return currentCones_;
}

bool Tracker::hasData() const {
  return !currentCones_.cones.empty();
}