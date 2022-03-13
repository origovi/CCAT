#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "structures/Params.hpp"
#include <as_msgs/Observation.h>
#include <vector>
#include <ros/ros.h>

class Tracker {
 private:
  /**
   * PRIVATE CONSTRUCTOR ANC DESTRUCTOR
   */

  Tracker();

  /**
   * PRIVATE ATTRIBUTES
   */
  
  ros::NodeHandle *nh_;
  Params::Tracker params_;
  
  /**
   * PRIVATE METHODS
   */

 public:
  ~Tracker();

  /**
   * PUBLIC METHODS
   */

  /* Singleton pattern */

  static Tracker &getInstance();
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /* Init */

  void init(ros::NodeHandle *const &nh, const Params::Tracker &params);
  
  /* Callbacks */

  /* Getters */

};

#endif // TRACKER_HPP