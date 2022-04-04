#include "structures/Tracking.hpp"

/**
 * CONSTRUCTORS
 */

Tracking::Tracking(const Cone &cone, const size_t &id) : id(id), valid_(false) {
  position_ = cone.observation->centroid_global;
  closestDist_.first = cone.observation->distToCar;
  closestDist_.second = cone.type;
}

/**
 * DESTRUCTORS
 */

Tracking::~Tracking() {}

/**
 * PRIVATE METHODS
 */

const uint8_t &Tracking::getConeType() {
  if (!valid_) {
    std::map<Cone::Type, double> heuristics;
    double temporalX = 0.1;
    double deltaX = 0.9 / 30;
    for (auto it = considerationsQ_.crbegin(); it != considerationsQ_.crend(); it++) {
      double temporalY = -log(std::min(temporalX, 1.0));
      double heuristic = it->matchingDist * it->distToCar * temporalY;
      if (heuristics.find(it->type) == heuristics.end()) {
        heuristics[it->type] = heuristic;
      } else
        heuristics[it->type] += heuristic;
      temporalX += deltaX;
    }
    // Find the one with best heuristic
    Cone::Type bestType = Cone::Type::None;
    double bestHeur = -1;
    for (auto it = heuristics.begin(); it != heuristics.end(); it++) {
      if (bestHeur == -1 or bestHeur > it->second) {
        bestHeur = it->second;
        bestType = it->first;
      }
    }
    type_ = bestType;
  }

  return static_cast<uint8_t>(type_);
}

/**
 * PUBLIC METHODS
 */

void Tracking::addCone(const Cone &cone, const double &distSqToPosition) {
  valid_ = false;
  position_ = cone.observation->centroid_global;
  if (cone.observation->distToCar < closestDist_.first and bool(cone)) {
    closestDist_.first = cone.observation->distToCar;
    closestDist_.second = cone.type;
  }

  // Considerations
  considerationsM_[cone.type].emplace_back(cone.observation->distToCar, cone.matchingDist, cone.type);
  considerationsQ_.emplace_back(cone.observation->distToCar, cone.matchingDist, cone.type);
  if (considerationsQ_.size() > 100) {
    considerationsQ_.pop_front();
  }
}

/* Getters */
const Point &Tracking::position() const {
  return position_;
}

const as_msgs::Cone &Tracking::getASCone(const Eigen::Affine3d &carTf) {
  as_msgs::Cone res;
  res.id = id;
  res.position_global = position_.gmPoint();
  // res.position_global.y = -res.position_global.y;
  res.position_base_link = position_.transformed(carTf).gmPoint();
  // res.position_base_link.y = -res.position_base_link.y;
  res.type = getConeType();
  return res;
}