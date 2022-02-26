#ifndef MATCHER_HPP
#define MATCHER_HPP

#include "Params.hpp"
#include <as_msgs/Observation.h>
#include <vector>

class Matcher {
 private:
  /**
   * CONSTRUCTORS
   */
  Matcher(const Params::Matcher &params);

  /**
   * DESTRUCTORS
   */
  ~Matcher();

  /**
   * PRIVATE ATTRIBUTES
   */
  const Params::Matcher params_;
  
  
  /**
   * PRIVATE METHODS
   */
 public:
  /**
   * PUBLIC METHODS
   */
  /* Singleton pattern */
  static Matcher &getInstance(const Params::Matcher &params);
  Matcher(Matcher const &) = delete;
  void operator=(Matcher const &) = delete;

  /* Callbacks */

  /* Getters */

};

#endif // MATCHER_HPP