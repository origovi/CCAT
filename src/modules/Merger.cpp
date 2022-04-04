#include "modules/Merger.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */
Merger::Merger() {}
Merger::~Merger() {}

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */

/* Singleton pattern */

Merger &Merger::getInstance() {
  static Merger merger;
  return merger;
}

/* Init */

void Merger::init(ros::NodeHandle *const &nh, const Params::Merger &params) {
  nh_ = nh;
  params_ = params;
}

void Merger::run(const std::vector<Cone> &conesLeft, const std::vector<Cone> &conesRight) {
  std::map<size_t, const Cone*> projCones;

  // We know that two cones will be the same because they will have the same
  // observation-id, we only need to find those that have the same id.
  // The Cone with a better (smaller) dist will prevail
  for (const Cone& cone : conesLeft) {
    auto it = projCones.find(cone.observation->id);
    if (it == projCones.end()) {
      projCones.insert({cone.observation->id, &cone});
    }
    else if (it->second->matchingDist > cone.matchingDist) {
      it->second = &cone;
    }
  }
  for (const Cone& cone : conesRight) {
    auto it = projCones.find(cone.observation->id);
    if (it == projCones.end()) {
      projCones.insert({cone.observation->id, &cone});
    }
    else if (!bool(it->second) or it->second->matchingDist > cone.matchingDist) {
      it->second = &cone;
    }
  }

  currentCones_.resize(projCones.size());
  size_t currConesInd = 0;
  for (const auto &projConesPair : projCones) {
    currentCones_[currConesInd++] = *projConesPair.second;
  }
}

/* Callbacks */

/* Getters */

const std::vector<Cone> &Merger::getData() const {
  return currentCones_;
}