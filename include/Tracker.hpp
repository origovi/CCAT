#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "Params.hpp"
#include <as_msgs/Observation.h>
#include <vector>

class Tracker {
 private:
  /**
   * CONSTRUCTORS
   */
  Tracker(const Params::Tracker &params);

  /**
   * DESTRUCTORS
   */
  ~Tracker();

  /**
   * PRIVATE ATTRIBUTES
   */
  const Params::Tracker params_;
  
  
  /**
   * PRIVATE METHODS
   */
 public:
  /**
   * PUBLIC METHODS
   */
  /* Singleton pattern */
  static Tracker &getInstance(const Params::Tracker &params);
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /* Callbacks */

  /* Getters */

};

#endif // TRACKER_HPP