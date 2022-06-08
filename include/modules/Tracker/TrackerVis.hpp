#ifndef MODULES_TRACKER_TRACKERVIS_HPP
#define MODULES_TRACKER_TRACKERVIS_HPP

#include "structures/Params.hpp"
#include "structures/Cone.hpp"
#include <as_msgs/ConeArray.h>
#include "utils/Visualization.hpp"

/**
 * @brief Visualization class for the Tracker module.
 * 
 */
class TrackerVis : public Visualization {
 private:
  Params::Tracker params_;
  void paintMarkerFromType(visualization_msgs::Marker &m, const uint8_t &type);

 public:
  void init(const Params::Tracker &params);
  void publishMergedMarkers(const std::vector<ConeUpdate> &coneUpdates, const std::map<size_t, Cone> &cones);
  void publishFinalMarkers(const as_msgs::ConeArray &currentCones);
};

#endif  // MODULES_TRACKER_TRACKERVIS_HPP