#include "modules/Merger/Merger.hpp"

/**
 * CONSTRUCTORS AND DESTRUCTOR
 */
Merger::Merger() {}

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

void Merger::init(const Params::Merger &params) {
  params_ = params;
}

void Merger::run(const std::vector<ConeUpdate> &conesLeft, const std::vector<ConeUpdate> &conesRight) {
  std::map<size_t, const ConeUpdate*> projCones;

  // We know that two cones will be the same because they will have the same
  // observation-id, we only need to find those that have the same id.
  // The Cone with a better (smaller) matching dist will prevail, all other
  // metrics will be ignored.
  for (const ConeUpdate& coneUpdate : conesLeft) {
    auto it = projCones.find(coneUpdate.id);
    if (it == projCones.end()) {
      projCones.insert({coneUpdate.id, &coneUpdate});
    }
    else if (it->second->matchingDist > coneUpdate.matchingDist) {
      it->second = &coneUpdate;
    }
  }
  for (const ConeUpdate& coneUpdate : conesRight) {
    auto it = projCones.find(coneUpdate.id);
    if (it == projCones.end()) {
      projCones.insert({coneUpdate.id, &coneUpdate});
    }
    else if (!bool(it->second) or it->second->matchingDist > coneUpdate.matchingDist) {
      it->second = &coneUpdate;
    }
  }

  currentCones_.clear();
  currentCones_.reserve(projCones.size());
  for (const auto &projConesPair : projCones) {
    currentCones_.push_back(*projConesPair.second);
  }
}

/* Callbacks */

/* Getters */

const std::vector<ConeUpdate> &Merger::getData() const {
  return currentCones_;
}