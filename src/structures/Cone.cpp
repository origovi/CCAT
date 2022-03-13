#include "structures/Cone.hpp"

/**
 * CONSTRUCTORS
 */

Cone::Cone() { type = None; }

Cone::Cone(const Observation &observation) : Observation(observation) { type = None; }

/**
 * DESTRUCTORS
 */

Cone::~Cone() {}

/**
 * PUBLIC METHODS
 */

void Cone::setTypeFromAsMsgs(const uint8_t &type) {
  this->type = static_cast<Type>(type);
}
