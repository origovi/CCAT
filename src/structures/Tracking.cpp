#include "structures/Tracking.hpp"

/**
 * CONSTRUCTORS
 */

Tracking::Tracking(const Cone &cone, const size_t &id) : id(id) {
  position_ = cone.observation->centroid_global;
  closestDist_.first = cone.observation->distToCar;
  closestDist_.second = cone.type;
}

/**
 * DESTRUCTORS
 */

Tracking::~Tracking() {}

/**
 * PUBLIC METHODS
 */

void Tracking::addCone(const Cone &cone, const double &distSqToPosition) {
  position_ = cone.observation->centroid_global;
  if (cone.observation->distToCar < closestDist_.first and cone.type != Cone::None) {
    closestDist_.first = cone.observation->distToCar;
    closestDist_.second = cone.type;
  }
}

/* Getters */
const Point &Tracking::position() const {
  return position_;
}

as_msgs::Cone Tracking::getASCone(const Eigen::Affine3d &carTf) const {
  as_msgs::Cone res;
  res.id = id;
  res.position_global = position_.gmPoint();
  res.position_base_link = position_.transformed(carTf).gmPoint();
  res.type = static_cast<uint8_t>(closestDist_.second);
  return res;
}