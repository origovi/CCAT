#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <as_msgs/ConeArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "structures/Cone.hpp"
#include "structures/Params.hpp"
#include "structures/Tracking.hpp"
#include "structures/KDTree.hpp"
#include "utilities/conversions.hpp"

class Tracker {
 private:
  /**
   * PRIVATE CONSTRUCTOR ANC DESTRUCTOR
   */

  Tracker();

  /**
   * PRIVATE ATTRIBUTES
   */

  ros::NodeHandle *nh_;
  Params::Tracker params_;
  ros::Publisher markerBaseLinkPub_, markerGlobalPub_;

  size_t lastId_;
  std::list<Tracking> trackings_;

  as_msgs::ConeArray currentCones_;

  /**
   * PRIVATE METHODS
   */
  void updateCurrentCones(const Eigen::Affine3d &carTf);
  void publishMarkers() const;

  void getTrackingPoints(std::vector<Point> &points, std::vector<Tracking*> &trackingPtrs);

 public:
  ~Tracker();

  /**
   * PUBLIC METHODS
   */

  /* Singleton pattern */

  static Tracker &getInstance();
  Tracker(Tracker const &) = delete;
  void operator=(Tracker const &) = delete;

  /* Init */

  void init(ros::NodeHandle *const &nh, const Params::Tracker &params);

  void run(const std::vector<Cone> &cones, const Eigen::Affine3d &carTf);

  /* Callbacks */

  /* Getters */

  const as_msgs::ConeArray &getData() const;
  bool hasData() const;
};

#endif  // TRACKER_HPP