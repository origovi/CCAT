#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <as_msgs/ConeArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "structures/Cone.hpp"
#include "structures/Params.hpp"
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

  as_msgs::ConeArray currentCones_;

  /**
   * PRIVATE METHODS
   */

  void publishMarkers() const;

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

  void run(const std::vector<Cone> &cones);

  /* Callbacks */

  /* Getters */

  const as_msgs::ConeArray &getData() const;
  bool hasData() const;
};

#endif  // TRACKER_HPP