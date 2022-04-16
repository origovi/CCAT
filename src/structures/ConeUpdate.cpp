#include "structures/ConeUpdate.hpp"

/**
 * CONSTRUCTORS
 */

ConeUpdate::ConeUpdate(const size_t &id) : id(id), matchingDist(CONE_DEFAULT_MATCH_DIST) {
  this->type = NONE;
  this->operation = ADD;
}

ConeUpdate::ConeUpdate(const size_t &id, const uint8_t &bbType, const double &matchingDist) : id(id), matchingDist(matchingDist) {
  setTypeFromAsMsgs(bbType);
  this->operation = ADD;
}

/**
 * PUBLIC METHODS
 */

ConeUpdate::operator bool() const { return type != NONE; };

void ConeUpdate::setTypeFromAsMsgs(const uint8_t &bbType) { this->type = static_cast<Type>(bbType); }

uint8_t ConeUpdate::typeToAsMsgs() const {
  return static_cast<uint8_t>(type);
}