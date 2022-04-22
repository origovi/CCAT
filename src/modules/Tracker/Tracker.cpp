#include "modules/Tracker/Tracker.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */

Tracker::Tracker() {
  lastId_ = 0;
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
}

void Tracker::accumulate(const std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> &data) {
  if (cones_.empty()) {
    for (const Observation &obs : data.first) {
      cones_.emplace_hint(cones_.end(), std::piecewise_construct, std::forward_as_tuple(lastId_), std::forward_as_tuple(obs, lastId_));
      lastId_++;
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
        cones_.emplace_hint(cones_.end(), std::piecewise_construct, std::forward_as_tuple(lastId_), std::forward_as_tuple(obs, lastId_));
        lastId_++;
      }
    }
  }

  // Update the Observations to local space
  for (std::map<size_t, Cone>::iterator it = cones_.begin(); it != cones_.end(); it++) {
    it->second.updateLocal(data.second);
  }
}

void Tracker::run(const std::vector<ConeUpdate> &coneUpdates) {
  for (const ConeUpdate &coneUpdate : coneUpdates) {
    std::map<size_t, Cone>::iterator it = cones_.find(coneUpdate.id);
    if (it != cones_.end()) {
      it->second.update(coneUpdate);
    }
    else {
      std::cout << "achtung" << std::endl << coneUpdate.id << std::endl;
      for (auto it = cones_.begin(); it != cones_.end(); it++) {
        std::cout << it->first << std::endl;
      } 
    }
  }

  // Update the current cones
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

const as_msgs::ConeArray &Tracker::getData() {
  // Publish markers, if debug
  if (params_.debug) {
    vis_.publishMarkers(currentCones_);
  }
  return currentCones_;
}

bool Tracker::hasData() const {
  return !currentCones_.cones.empty();
}