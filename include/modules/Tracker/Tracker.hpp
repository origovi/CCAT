/**
 * @file Tracker.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Tracker module.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_TRACKER_TRACKER_HPP
#define MODULES_TRACKER_TRACKER_HPP

#include <as_msgs/ConeArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "modules/Tracker/TrackerVis.hpp"
#include "structures/Cone.hpp"
#include "structures/ConeUpdate.hpp"
#include "structures/Params.hpp"
#include "utils/KDTree.hpp"

/**
 * @brief The Tracker module is the only module in which cones are stored
 * between iterations. The objective of this module is to keep track of all
 * observations and decide which one should be output and the type of it.
 */
class Tracker {
 private:
  /* -------------------------- Private Constructor ------------------------- */

  Tracker();

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Tracker, it includes all necessary
   * paramters that will be used in the module.
   */
  Params::Tracker params_;

  /**
   * @brief Next id to give to a Cone.
   */
  size_t nextId_;

  /**
   * @brief Main memory object, here the Cone(s) are stored and tracked with
   * the given id.
   */
  std::map<size_t, Cone> cones_;

  /**
   * @brief Current valid cones in the output format (as_msgs), later on,
   * another ROS node could read this data.
   */
  as_msgs::ConeArray currentCones_;

  /**
   * @brief Number of cones seen in last iteration
   */
  size_t actualConeNumber_;

  /**
   * @brief The Visualization object for the Tracker, it will allow us to
   * publish all debug messages.
   */
  TrackerVis vis_;

  /* ---------------------------- Private Methods --------------------------- */

  /**
   * @brief Transform a vector of Cone(s) into a vector of Point(s), useful when
   * creating a k-d tree (we need the raw points).
   * 
   * @param[in,out] points 
   * @param[in] trackingPtrs 
   */
  void getTrackingPoints(std::vector<Point> &points, std::vector<Cone *> &trackingPtrs);

 public:
  /* --------------------------- Singleton Pattern -------------------------- */

  static Tracker &getInstance();
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /**
   * @brief It initializes the module.
   * 
   * @param[in] params The params that will be used by the Tracker
   */
  void init(const Params::Tracker &params);

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief In charge of making the function of the Accumulator fictional
   * module, add new observations with the historic. To do so, for every new
   * Observation, it finds the closest position and assumes it corresponds the
   * same cone, if this distance is too high, a new cone is assumed to be seen.
   * 
   * @param[in] data are the new Observation(s) and the new car state
   */
  void accumulate(const std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> &data);

  /**
   * @brief Main function of the Tracker class, it runs all the pipeline and
   * updates the currentCones_ attribute as a result.
   * To do so, a statistical heuristically-ponderated model is employed.
   * 
   * @param[in] coneUpdates are the ConeUpdate(s) obtained from the Merger
   * module
   */
  void run(const std::vector<ConeUpdate> &coneUpdates);

  /**
   * @brief Getter for all the Observation(s) contained by the module.
   * 
   * @return a vector of shared pointers to Observation(s)
   */
  std::vector<Observation::Ptr> getObservations() const;

  /**
   * @brief Getter for the number of actual cones required for FSG.
   * 
   * @return the number of cones seen in actual's iteration
   */
  const size_t &getActualNumCones() const;

  /**
   * @brief Getter for the number of total cones required for FSG.
   * 
   * @return the number of cones seen historically
   */
  size_t getTotalNumCones() const;

  /**
   * @brief Returns the data (cones correctly classified) in as_msgs format-
   * 
   * @return a vector of cones
   */
  const as_msgs::ConeArray &getData();

  /**
   * @brief Checks if the module has valid data.
   * 
   * @return true 
   * @return false 
   */
  bool hasData() const;
};

#endif  // MODULES_TRACKER_TRACKER_HPP