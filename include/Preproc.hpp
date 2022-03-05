#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <as_msgs/Observation.h>
#include <as_msgs/ObservationArray.h>
#include <ros/ros.h>

#include <list>
#include <unordered_map>
#include <vector>

#include "KDTree.hpp"
#include "Observation.hpp"
#include "Params.hpp"
#include "Point.hpp"
#include "conversions.hpp"

class Preproc {
 private:
  /**
   * CONSTRUCTORS
   */
  Preproc(const Params::Preproc &params);

  /**
   * DESTRUCTORS
   */
  ~Preproc();

  /**
   * PRIVATE ATTRIBUTES
   */
  const Params::Preproc params_;
  bool hasData_;
  KDTree *observationsTree_;
  std::vector<Observation> currentObservations_;

  /**
   * PRIVATE METHODS
   */
  Observation centroidObs(const std::list<size_t> &points, const std::vector<Observation> &allObs) const;
  std::list<size_t> possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const;
  std::vector<Observation> preprocess(const as_msgs::ObservationArray &observations) const;

 public:
  /**
   * PUBLIC METHODS
   */
  /* Singleton pattern */
  static Preproc &getInstance(const Params::Preproc &params);
  Preproc(Preproc const &) = delete;
  void operator=(Preproc const &) = delete;

  /* Callbacks */
  void obsCallback(const as_msgs::ObservationArray &newObservations);

  /* Getters */
  const std::vector<Observation> &getCurrentObservations() const;
  const bool &hasData() const;
};

#endif  // PREPROC_HPP