#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <algorithm>

#include "as_msgs/Cone.h"
#include "structures/Cone.hpp"
#include "structures/Point.hpp"

class Tracking {
 private:
  struct Consideration {
    Cone::Type type;
    double heuristic;
    Consideration(const Cone::Type &type, const double &heuristic) : type(type), heuristic(heuristic) {}
    static bool comparer(const Consideration &c1, const Consideration &c2) {return c1.heuristic < c2.heuristic;}
  };

  Point position_;
  std::vector<Consideration> heap_;

  Cone::Type type_;
  
  static double getHeuristic(const Cone &cone, const double &distSqToOldPos);
 public:
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  Tracking(const Cone &cone, const size_t &id);

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