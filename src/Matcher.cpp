#include "Matcher.hpp"

/**
 * CONSTRUCTORS
 */
Matcher::Matcher(const Params::Matcher &params) : params_(params) {}

/**
 * DESTRUCTORS
 */
Matcher::~Matcher() {}

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */
/* Singleton pattern */
Matcher &Matcher::getInstance(const Params::Matcher &params) {
  static Matcher matcher(params);
  return matcher;
}
/* Callbacks */

/* Getters */
