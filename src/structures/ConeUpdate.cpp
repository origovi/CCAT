#include "structures/ConeUpdate.hpp"

/**
 * CONSTRUCTORS
 */

ConeUpdate::ConeUpdate(const size_t &id, const double &distToCameraPlane, const double &distToClosestMatch) : id(id), probBB(CONE_DEFAULT_PROB_BB), matchingDist(CONE_DEFAULT_MATCH_DIST), distToCameraPlane(distToCameraPlane), distToClosestMatch(distToClosestMatch) {
  this->type = NONE;
}

ConeUpdate::ConeUpdate(const size_t &id, const uint8_t &bbType, const float &probBB, const double &matchingDist, const double &distToCameraPlane) : id(id), probBB(probBB), matchingDist(matchingDist), distToCameraPlane(distToCameraPlane), distToClosestMatch(CONE_DEFAULT_DIST_TO_CLOSEST_MATCH) {
  setTypeFromAsMsgs(bbType);
}

/**
 * PUBLIC METHODS
 */

ConeUpdate::operator bool() const { return type != NONE; };

void ConeUpdate::setTypeFromAsMsgs(const uint8_t &bbType) { this->type = static_cast<Type>(bbType); }

uint8_t ConeUpdate::typeToAsMsgs() const {
  return static_cast<uint8_t>(type);
}