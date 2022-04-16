#ifndef MERGER_HPP
#define MERGER_HPP

#include <ros/ros.h>

#include <vector>
#include <map>

#include "structures/ConeUpdate.hpp"
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

  std::vector<ConeUpdate> currentCones_;

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

  void init(const Params::Merger &params);

  /* Functions */

  void run(const std::vector<ConeUpdate> &conesLeft, const std::vector<ConeUpdate> &conesRight);

  /* Callbacks */

  /* Getters */

  const std::vector<ConeUpdate> &getData() const;
};

#endif  // MERGER_HPP