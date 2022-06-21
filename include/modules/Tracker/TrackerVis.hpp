/**
 * @file TrackerVis.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It provides all the necessary tools to the Tracker module to
 * visualize all the data and debug.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_TRACKER_TRACKERVIS_HPP
#define MODULES_TRACKER_TRACKERVIS_HPP

#include "structures/Params.hpp"
#include "structures/Cone.hpp"
#include <as_msgs/ConeArray.h>
#include "utils/Visualization.hpp"

/**
 * @brief Visualization class for the Tracker module.
 */
class TrackerVis : public Visualization {
 private:
  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Tracker, it includes all necessary
   * paramters that will be used in the module.
   */
  Params::Tracker params_;

  /**
   * @brief Changes the color and type of the marker \a m in order to make it
   * seem like a classified cone.
   * 
   * @param[out] m is the marker to be painted
   * @param[in] type the type to paint the marker
   */
  void paintMarkerFromType(visualization_msgs::Marker &m, const uint8_t &type);

 public:
  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief It initializes the submodule.
   * 
   * @param[in] params The params that will be used by the Tracker
   * (and TrackerVis)
   */
  void init(const Params::Tracker &params);

  /**
   * @brief Creates and publishes the markers corresponding to the merged cones
   * for debug purposes.
   * 
   * @param coneUpdates
   * @param cones the dictionary containing all the Cone(s) with their ids
   */
  void publishMergedMarkers(const std::vector<ConeUpdate> &coneUpdates, const std::map<size_t, Cone> &cones);

  /**
   * @brief Creates and publishes a set of markers from a given set of cones,
   * these markers can also show the cone's id above it.
   * 
   * @param currentCones the set of cones that will be published
   */
  void publishFinalMarkers(const as_msgs::ConeArray &currentCones);
};

#endif  // MODULES_TRACKER_TRACKERVIS_HPP