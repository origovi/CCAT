#include "structures/Cone.hpp"

/**
 * CONSTRUCTORS
 */

Cone::Cone() {
  type = None;
  operation = ADD;
}

Cone::Cone(const Observation &observation) : Observation(observation) {
  type = None;
  operation = ADD;
}

/**
 * DESTRUCTORS
 */

Cone::~Cone() {}

/**
 * PUBLIC METHODS
 */

void Cone::setTypeFromAsMsgs(const uint8_t &type) { this->type = static_cast<Type>(type); }
