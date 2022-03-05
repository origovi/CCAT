#include "Observation.hpp"

/**
 * CONSTRUCTORS
 */
Observation::Observation() : p(), confidence(0.0) {}
Observation::Observation(const float &x, const float &y, const float &z, const float &confidence) : p(x, y, z), confidence(confidence) {}
Observation::Observation(const as_msgs::Observation & obs) : p(obs.x, obs.y, obs.z), confidence(obs.confidence) {}

/**
 * DESTRUCTORS
 */
Observation::~Observation() {}

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */
Observation &Observation::operator+=(const Observation &o) {
  p += o.p;
  confidence += o.confidence;
  return *this;
}

Observation &Observation::operator-=(const Observation &o) {
  p -= o.p;
  confidence -= o.confidence;
  return *this;
}

Observation &Observation::operator/=(const int &num) {
  p /= num;
  confidence /= num;
  return *this;
}

/* Getters */
const double &Observation::at(const size_t &ind) const {
  return p.at(ind);
}

size_t Observation::size() const { return p.size(); }

/* Getters */
int Observation::getHash() const {
  return 0;
}
