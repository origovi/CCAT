#ifndef STRUCTURES_CONE_HPP
#define STRUCTURES_CONE_HPP

#include <Eigen/Geometry>
#include <algorithm>

#include "as_msgs/Cone.h"
#include "structures/ConeUpdate.hpp"
#include "structures/Observation.hpp"
#include "structures/Point.hpp"

class Cone {
 private:
  struct Consideration {
    ConeUpdate::Type type;
    double heuristic;
    Consideration(const ConeUpdate::Type &type, const double &heuristic) : type(type), heuristic(heuristic) {}
    static bool comparer(const Consideration &c1, const Consideration &c2) { return c1.heuristic < c2.heuristic; }
  };

  std::vector<Consideration> heap_;
  ConeUpdate::Type type_;
  bool valid_;

  static double getHeuristic(const ConeUpdate &coneUpdate, const double &distSqToOldPos);

 public:
  /**
   * CONSTRUCTOR
   */

  Cone(const Observation &obs, const size_t &id);

  /**
   * PUBLIC ATTRIBUTES
   */

  Observation::Ptr obs;
  const size_t id;

  /**
   * PUBLIC METHODS
   */

  void update(const ConeUpdate &coneUpdate);
  void updateObs(const Observation &obs, const double &distSqToPosition);
  void updateLocal(const Eigen::Affine3d &carTf);

  /* Getters */
  const Point &position_global() const;
  const bool &valid() const;
  as_msgs::Cone getASCone() const;
};

#endif  // STRUCTURES_CONE_HPP