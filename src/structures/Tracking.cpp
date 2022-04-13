#include "structures/Tracking.hpp"

/**
 * CONSTRUCTORS
 */

Tracking::Tracking(const Cone &cone, const size_t &id) : id(id) {
  heap_.reserve(20);
  position_ = cone.observation->centroid_global;
  type_ = cone.type;
}


/**
 * PRIVATE METHODS
 */

double Tracking::getHeuristic(const Cone &cone, const double &distSqToOldPos) {
  std::cout << cone.type << " " << cone.matchingDist << " " << cone.distToCameraPlane << " " << cone.matchingDist*cone.distToCameraPlane << " " << 1/(cone.matchingDist*cone.distToCameraPlane) << std::endl;
  return 1.0/(cone.matchingDist*cone.distToCameraPlane);
}

/**
 * PUBLIC METHODS
 */

void Tracking::addCone(const Cone &cone, const double &distSqToOldPos) {
  position_ = cone.observation->centroid_global;
  if (cone.type == Cone::None) return;
  double coneHeuristic = getHeuristic(cone, distSqToOldPos);
  if (heap_.size() >= 20) {
    
    // Find the element with min heuristic in the vector
    // and replace it if the heuristic is bigger.
    auto min_it = std::min_element(heap_.begin(), heap_.end(), Consideration::comparer);
    
    // Replace the consideration
    if (coneHeuristic > min_it->heuristic) {
      *min_it = Consideration(cone.type, coneHeuristic);
    }
  } else {
    heap_.emplace_back(cone.type, coneHeuristic);
  }

  // Update "type_" summing all the heuristics in the vector,
  // the type with a bigger value wins.
  std::map<Cone::Type, double> votation;
  for (const Consideration &cons : heap_) {
    if (votation.find(cons.type) == votation.end())
      votation[cons.type] = cons.heuristic;
    else
      votation[cons.type] += cons.heuristic;
  }

  Cone::Type biggestType = Cone::Type::None;
  double biggestHeuristic = -1.0;
  for (const std::pair<Cone::Type, double> &vote : votation) {
    if (biggestHeuristic < vote.second and vote.first != Cone::None) {
      biggestHeuristic = vote.second;
      biggestType = vote.first;
    } 
  }
  type_ = biggestType;
}

/* Getters */

const Point &Tracking::position() const {
  return position_;
}

as_msgs::Cone Tracking::getASCone(const Eigen::Affine3d &carTf) const {
  as_msgs::Cone res;
  res.id = id;
  res.position_global = position_.gmPoint();
  // res.position_global.y = -res.position_global.y;
  res.position_base_link = position_.transformed(carTf).gmPoint();
  // res.position_base_link.y = -res.position_base_link.y;
  res.type = static_cast<uint8_t>(type_);
  return res;
}