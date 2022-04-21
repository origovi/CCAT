#include "structures/Cone.hpp"

/**
 * CONSTRUCTORS
 */

Cone::Cone(const Observation &obs, const size_t &id) : id(id) {
  heap_.reserve(20);
  this->obs = std::make_shared<Observation>(obs);
  type_ = ConeUpdate::NONE;
}

/**
 * PRIVATE METHODS
 */

double Cone::getHeuristic(const ConeUpdate &coneUpdate, const double &distSqToOldPos) {
  std::cout << coneUpdate.type << " " << coneUpdate.matchingDist << " " << coneUpdate.distToCameraPlane << " " << coneUpdate.matchingDist * coneUpdate.distToCameraPlane << " " << 1 / (coneUpdate.matchingDist * coneUpdate.distToCameraPlane) << std::endl;
  return 1.0 / (coneUpdate.matchingDist * coneUpdate.distToCameraPlane);
}

/**
 * PUBLIC METHODS
 */

void Cone::update(const ConeUpdate &coneUpdate) {
  if (coneUpdate.type == ConeUpdate::NONE) return;
  double coneHeuristic = getHeuristic(coneUpdate, 0.0);
  if (heap_.size() >= 20) {
    // Find the element with min heuristic in the vector
    // and replace it if the heuristic is bigger.
    auto min_it = std::min_element(heap_.begin(), heap_.end(), Consideration::comparer);

    // Replace the consideration
    if (coneHeuristic > min_it->heuristic) {
      *min_it = Consideration(coneUpdate.type, coneHeuristic);
    }
  } else {
    heap_.emplace_back(coneUpdate.type, coneHeuristic);
  }

  // Update "type_" summing all the heuristics in the vector,
  // the type with a bigger value wins.
  std::map<ConeUpdate::Type, double> votation;
  for (const Consideration &cons : heap_) {
    if (votation.find(cons.type) == votation.end())
      votation[cons.type] = cons.heuristic;
    else
      votation[cons.type] += cons.heuristic;
  }

  ConeUpdate::Type biggestType = ConeUpdate::Type::NONE;
  double biggestHeuristic = -1.0;
  for (const std::pair<ConeUpdate::Type, double> &vote : votation) {
    if (biggestHeuristic < vote.second and vote.first != ConeUpdate::NONE) {
      biggestHeuristic = vote.second;
      biggestType = vote.first;
    }
  }
  type_ = biggestType;
}

void Cone::updateObs(const Observation &obs, const double &distSqToPosition) {
  *this->obs = obs;
}

void Cone::updateLocal(const Eigen::Affine3d &carTf) {
  obs->updateLocal(carTf);
}

/* Getters */

const Point &Cone::position_global() const {
  return obs->centroid_global;
}

as_msgs::Cone Cone::getASCone() const {
  as_msgs::Cone res;
  res.id = static_cast<uint32_t>(id);
  res.position_global = position_global().gmPoint();
  res.position_baseLink = obs->temp.centroid_local.gmPoint();
  res.type = static_cast<uint8_t>(type_);
  return res;
}