/**
 * @file Preproc.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Preproc module.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_PREPROC_PREPROC_HPP
#define MODULES_PREPROC_PREPROC_HPP

#include <as_msgs/Observation.h>
#include <as_msgs/ObservationArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <list>
#include <vector>

#include "modules/Matcher/Matcher.hpp"
#include "modules/Preproc/PreprocVis.hpp"
#include "structures/Observation.hpp"
#include "structures/Params.hpp"
#include "structures/Point.hpp"
#include "utils/KDTree.hpp"

/**
 * @brief This module preprocesses all data that arrives in each iteration.
 * The aim of this module is to filter and clean the data so all the other
 * modules can use it directly.
 */
class Preproc {
 private:
  /* -------------------------- Private Constructor ------------------------- */

  Preproc();

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Preproc, it includes all necessary
   * paramters that will be used in the module.
   */
  Params::Preproc params_;

  /**
   * @brief This iteration's preprocessed observations. Can be get with
   * Preproc::getData().
   */
  std::vector<Observation> currentObservations_;

  /**
   * @brief Original stamp of currentObservations_. It is used to determine
   * if an incoming data is older than current data. If so,
   * currentObservations_ is not modified.
   * 
   */
  ros::Time currentObservationsStamp_;

  /**
   * @brief Current bounding boxes' data.
   */
  geometry_msgs::PoseArray::ConstPtr leftBBs_, rightBBs_;

  /**
   * @brief Current car transform (global-local) obtained from the car's position.
   * 
   */
  Eigen::Affine3d carTf_;

  /**
   * @brief The Visualization object for the Preproc, it will allow us to
   * publish all debug messages.
   */
  PreprocVis vis_;

  /* ---------------------------- Private Methods --------------------------- */

  /**
   * @brief Recursive function that finds all cone positions that form the
   * cluster in which cone with index \a pointIndex belongs.
   * 
   * @param[in] pointIndex is the index of the corresponding Observation in the
   * input observations vector
   * @param[in] observationsKDT a k-d tree that contain all observation
   * positions
   * @param[in] allObs is a vector containing all Observation(s) (detected in
   * the global 3D map)
   * @param[in,out] visited contains which points have been visited
   * (in a cluster) and which ones are free
   * @return a list of pointers with all Observation(s) in the same cluster as
   * \a pointIndex and that have not been visited yet
   */
  std::list<const Observation *> possiblesSamePoint(const size_t &pointIndex, const KDTree &observationsKDT, const std::vector<Observation> &allObs, std::vector<bool> &visited) const;

  /**
   * @brief Preprocess all data and updates currentObservations_.
   * 
   * @param[in] observations are the global map detected cones (input data) to
   * the Preproc module.
   */
  void preprocess(const as_msgs::ObservationArray &observations);

 public:
  /* --------------------------- Singleton Pattern -------------------------- */

  static Preproc &getInstance();
  Preproc(Preproc const &) = delete;
  void operator=(Preproc const &) = delete;

  /**
   * @brief It initializes the module.
   * 
   * @param[in] params The params that will be used by the Preproc
   */
  void init(const Params::Preproc &params);

  /**
   * @brief Main function of the Merger class, if all the below conditions
   * are met, it calles Preproc::preprocess() and updates the current data
   * attributes:
   * - \a newObservations not null
   * - \a newObservations stamp is newer than \a currentObservationsStamp_
   * 
   * @param[in] newObservations Newest observations from the global 3D map
   * @param[in] odom Newest car's odometry (pose 6D)
   * @param[in] leftBBs Newest bounding boxes from left camera
   * @param[in] rightBBs Newest bounding boxes from right camera
   */
  void run(const as_msgs::ObservationArray::ConstPtr &newObservations,
           const Eigen::Affine3d &odom,
           const geometry_msgs::PoseArray::ConstPtr &leftBBs,
           const geometry_msgs::PoseArray::ConstPtr &rightBBs);

  /**
   * @brief Returns the bounding boxes of the camera specified by \a which.
   * 
   * @param[in] which 
   * @return A pointer to the specified bounding boxes object
   */
  const geometry_msgs::PoseArray::ConstPtr &getBBs(const Matcher::Which &which) const;

  /**
   * @brief Returns the latest observation vector and the
   * latest car transform (global-local).
   * 
   * @return A tuple containing the data
   */
  std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> getData() const;

  /**
   * @brief A getter for the car transform (global-local)
   * 
   * @return the car transform object
   */
  const Eigen::Affine3d &getCarTf() const;
};

#endif  // MODULES_PREPROC_PREPROC_HPP