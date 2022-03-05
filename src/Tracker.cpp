#include "Tracker.hpp"

/**
 * CONSTRUCTORS
 */
Tracker::Tracker(const Params::Tracker &params) : params_(params) {}

/**
 * DESTRUCTORS
 */
Tracker::~Tracker() {}

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */
/* Singleton pattern */
Tracker &Tracker::getInstance(const Params::Tracker &params) {
  static Tracker matcher(params);
  return matcher;
}
/* Callbacks */

/* Getters */
