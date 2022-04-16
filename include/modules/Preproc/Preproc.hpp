#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <as_msgs/Observation.h>
#include <as_msgs/ObservationArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <list>
#include <vector>

#include "modules/Matcher/Matcher.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "structures/Point.hpp"
#include "utils/KDTree.hpp"
#include "utils/conversions.hpp"

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
  geometry_msgs::PoseArray::ConstPtr leftBBs_, rightBBs_;
  Eigen::Affine3d carTf_;

  /**
   * PRIVATE METHODS
   */

  std::list<const Observation *> possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const;
  void preprocess(const as_msgs::ObservationArray &observations);

 public:
  /**
   * PUBLIC METHODS
   */

  /* Singleton pattern */

  static Preproc &getInstance();
  Preproc(Preproc const &) = delete;
  void operator=(Preproc const &) = delete;

  /* Init */

  void init(const Params::Preproc &params);

  void reset();

  /* Callbacks */

  void callback(const as_msgs::ObservationArray::ConstPtr &newObservations, const nav_msgs::Odometry::ConstPtr &carPos,
                const geometry_msgs::PoseArray::ConstPtr &leftDetections,
                const geometry_msgs::PoseArray::ConstPtr &rightDetections);

  /* Getters */
  geometry_msgs::PoseArray::ConstPtr getBBs(const Matcher::Which &which) const;
  std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> getData() const;
  const bool &hasData() const;
  const Eigen::Affine3d &getCarTf() const;
};

#endif  // PREPROC_HPP