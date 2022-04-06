#include "structures/Cone.hpp"

/**
 * CONSTRUCTORS
 */


Cone::Cone(const Observation::Ptr &_observation, const double &_matchingDist) {
  observation = _observation;
  type = None;
  operation = ADD;
  matchingDist = _matchingDist;
  distToCameraPlane = _observation->centroid_transformed.z;
}

/**
 * PUBLIC METHODS
 */

Cone::operator bool() const { return type != None; };

void Cone::setTypeFromAsMsgs(const uint8_t &type) { this->type = static_cast<Type>(type); }

uint8_t Cone::typeToAsMsgs() const {
  return static_cast<uint8_t>(type);
}