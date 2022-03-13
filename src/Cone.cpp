#include "Cone.hpp"

/**
 * CONSTRUCTORS
 */
Cone::Cone() {
  type = None;
}

Cone::Cone(const Observation &observation) : Observation(observation) {
  type = None;
}

/**
 * DESTRUCTORS
 */
Cone::~Cone() {}

/**
 * PROTECTED METHODS
 */
