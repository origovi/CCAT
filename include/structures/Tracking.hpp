#ifndef TRACKING_HPP
#define TRACKING_HPP

#include "as_msgs/Cone.h"
#include "structures/Cone.hpp"
#include "structures/Point.hpp"

class Tracking {
 private:
  using Metric = typename std::pair<double, Cone::Type>;

  Point position_;
  Metric closestDist_, closestMatch_;

 public:
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  Tracking(const Cone &cone, const size_t &id);
  ~Tracking();

  /**
   * PUBLIC ATTRIBUTES
   */

  const size_t id;

  /**
   * PUBLIC METHODS
   */

  void addCone(const Cone &cone, const double &distSqToPosition);

  /* Getters */
  const Point &position() const;
  as_msgs::Cone getASCone(const Eigen::Affine3d &carTf) const;
};

#endif  // TRACKING_HPP