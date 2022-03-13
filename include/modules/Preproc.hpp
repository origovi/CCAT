#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <as_msgs/Observation.h>
#include <as_msgs/ObservationArray.h>
#include <ros/ros.h>

#include <list>
#include <vector>

#include "structures/KDTree.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "structures/Point.hpp"
#include "utilities/conversions.hpp"

class Preproc {
 private:
  /**
   * PRIVATE CONSTRUCTOR AND DESTRUCTOR
   */

  Preproc();

  /**
   * PRIVATE ATTRIBUTES
   */

  ros::NodeHandle *nh_;
  Params::Preproc params_;
  bool hasData_;
  std::vector<Observation> currentObservations_;

  /**
   * PRIVATE METHODS
   */

  std::list<const Observation *> possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const;
  std::vector<Observation> preprocess(const as_msgs::ObservationArray &observations) const;

 public:

  /**
   * PUBLIC METHODS
   */

  ~Preproc();
  /* Singleton pattern */

  static Preproc &getInstance();
  Preproc(Preproc const &) = delete;
  void operator=(Preproc const &) = delete;

  /* Init */

  void init(ros::NodeHandle *const &nh, const Params::Preproc &params);

  /* Callbacks */

  void obsCallback(const as_msgs::ObservationArray &newObservations);

  /* Getters */

  const std::vector<Observation> &getCurrentObservations() const;
  const bool &hasData() const;
};

#endif  // PREPROC_HPP