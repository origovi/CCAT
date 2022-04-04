#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <map>
#include <deque>

#include "as_msgs/Cone.h"
#include "structures/Cone.hpp"
#include "structures/Point.hpp"

class Tracking {
 private:
  using Metric = typename std::pair<double, Cone::Type>;
  struct Consideration {
    const Cone::Type type;
    const double distToCar;
    const double matchingDist;
    Consideration(double distToCar, double matchingDist, const Cone::Type &type) : distToCar(distToCar), matchingDist(matchingDist), type (type) {}
  };

  Point position_;
  std::vector<Consideration> heap_;

  bool valid_;
  Cone::Type type_;

  Metric closestDist_, closestMatch_;
  std::deque<Consideration> considerationsQ_;
  std::map<Cone::Type, std::list<Consideration>> considerationsM_;
  
  const uint8_t &getConeType();
  
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
  const as_msgs::Cone &getASCone(const Eigen::Affine3d &carTf);
};

#endif  // TRACKING_HPP