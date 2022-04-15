#ifndef MERGER_HPP
#define MERGER_HPP

#include <ros/ros.h>

#include <vector>
#include <map>

#include "structures/Cone.hpp"
#include "structures/Params.hpp"

class Merger {
 private:
  /**
   * PRIVATE CONSTRUCTOR AND DESTRUCTOR
   */

  Merger();

  /**
   * PRIVATE ATTRIBUTES
   */

  Params::Merger params_;
  ros::NodeHandle *nh_;

  std::vector<Cone> currentCones_;

  /**
   * PRIVATE METHODS
   */

 public:
  /**
   * PUBLIC METHODS
   */

  /* Singleton pattern */

  static Merger &getInstance();
  Merger(Merger const &) = delete;
  void operator=(Merger const &) = delete;

  /* Init */

  void init(ros::NodeHandle *const &nh, const Params::Merger &params);

  /* Functions */

  void run(const std::vector<Cone> &conesLeft, const std::vector<Cone> &conesRight);

  /* Callbacks */

  /* Getters */

  const std::vector<Cone> &getData() const;
};

#endif  // MERGER_HPP