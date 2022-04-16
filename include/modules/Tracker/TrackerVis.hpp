#ifndef MODULES_TRACKER_TRACKERVIS_HPP
#define MODULES_TRACKER_TRACKERVIS_HPP

#include "structures/Params.hpp"
#include <as_msgs/ConeArray.h>
#include "utils/Visualization.hpp"

/**
 * @brief Visualization class for the Tracker module.
 * 
 */
class TrackerVis : public Visualization {
 private:
  Params::Tracker params_;

 public:
  void init(const Params::Tracker &params);
  void publishMarkers(const as_msgs::ConeArray &currentCones);
};

#endif  // MODULES_TRACKER_TRACKERVIS_HPP