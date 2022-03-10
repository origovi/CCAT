#include "Tracker.hpp"

/**
 * CONSTRUCTORS
 */
Tracker::Tracker() {}

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
Tracker &Tracker::getInstance() {
  static Tracker matcher;
  return matcher;
}

/* Init */
void Tracker::init(ros::NodeHandle *const &nh, const Params::Tracker &params) {
  nh_ = nh;
  params_ = params;
}
/* Callbacks */

/* Getters */
