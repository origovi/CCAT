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

class Tracker {
 private:
  /**
   * PRIVATE CONSTRUCTOR ANC DESTRUCTOR
   */

  Tracker();

  /**
   * PRIVATE ATTRIBUTES
   */

  Params::Tracker params_;

  size_t lastId_;
  std::map<size_t, Cone> cones_;

  as_msgs::ConeArray currentCones_;

  TrackerVis vis_;

  /**
   * PRIVATE METHODS
   */
  void updateCurrentCones();

  void getTrackingPoints(std::vector<Point> &points, std::vector<Cone *> &trackingPtrs);

 public:
  /**
   * PUBLIC METHODS
   */

  /* Singleton pattern */

  static Tracker &getInstance();
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /* Init */

  void init(const Params::Tracker &params);

  void accumulate(const std::pair<const std::vector<Observation> &, const Eigen::Affine3d &> &data);

  void run(const std::vector<ConeUpdate> &coneUpdates);

  /* Callbacks */

  /* Getters */
  std::vector<Observation::Ptr> getObservations() const;
  const as_msgs::ConeArray &getData();
  bool hasData() const;
};

#endif  // MODULES_TRACKER_TRACKER_HPP