#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <as_msgs/Observation.h>
#include <as_msgs/ObservationArray.h>
#include <ros/ros.h>

#include <list>
#include <unordered_map>
#include <vector>

#include "KDTree.hpp"
#include "Params.hpp"
#include "Reference.hpp"
#include "conversions.hpp"

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
  bool hasData_;
  KDTree *observationsTree_;
  std::vector<as_msgs::Observation> currentObservations_;
  std::unordered_map<int, Reference> observations_;

  /**
   * PRIVATE METHODS
   */
  Point centroidPoint(const std::list<size_t> &points, const std::vector<Point> &allPoints) const;
  std::list<size_t> possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Point> &allPoints, std::vector<bool> &visited) const;
  std::list<Point> preprocess(const as_msgs::ObservationArray &observations) const;

 public:
  /**
   * PUBLIC METHODS
   */
  /* Singleton pattern */
  static Tracker &getInstance(const Params::Tracker &params);
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /* Callbacks */
  void mapCallback(const as_msgs::ObservationArray &newObservations);

  /* Getters */
  const std::vector<as_msgs::Observation> &getCurrentObservations() const;
  const bool &hasData() const;
};

#endif  // TRACKER_HPP