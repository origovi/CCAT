#include "structures/Cone.hpp"

Params::Tracker::Cone Cone::params;
bool Cone::bothCams;
/**
 * CONSTRUCTORS
 */

Cone::Cone(const Observation &obs, const size_t &id) : id(id) {
  heapM_.reserve(params.heap_size);
  heap_.reserve(params.heap_size);
  this->obs = std::make_shared<Observation>(obs);
  this->obs->id = id;
  metadata_.type = ConeUpdate::NONE;
  metadata_.valid = true;
}

/**
 * PRIVATE METHODS
 */

double Cone::getHeuristicM(const ConeUpdate &coneUpdate, const double &distSqToOldPos) {
  return 1.0 / (std::max(coneUpdate.matchingDist, 2.0) * coneUpdate.distToCameraPlane);
}

double Cone::getHeuristic(const ConeUpdate &coneUpdate, const double &distSqToOldPos) {
  return 1.0 / coneUpdate.distToCameraPlane;
}

void Cone::updateMetadata() {
  // Normal heap
  int heap_votesM = 0;
  double heap_heurM = 0.0;

  int heap_votesNM = 0;
  double heap_heurNM = 0.0;

  for (const Consideration &cons : heap_) {
    if (cons.isMatched) {
      heap_votesM++;
      heap_heurM += cons.heuristic;
    } else {
      heap_votesNM++;
      heap_heurNM += cons.heuristic;
    }
  }
  double heap_meanHeur;
  bool heap_isMatched;
  if (heap_votesM > heap_votesNM*1e-4) {
    heap_isMatched = true;
    heap_meanHeur = heap_heurM / heap_votesM;
  } else {
    heap_isMatched = false;
    heap_meanHeur = heap_heurNM / heap_votesNM;
  }

  // Matched heap
  std::map<ConeUpdate::Type, std::pair<unsigned int, double>> heapM_votation;
  for (const ConsiderationM &cons : heapM_) {
    auto it = heapM_votation.find(cons.type);
    if (it == heapM_votation.end())
      heapM_votation[cons.type] = std::make_pair(1, cons.heuristic);
    else {
      it->second.first++;
      it->second.second += cons.heuristic;
    }
  }

  ConeUpdate::Type heapM_biggestType = ConeUpdate::Type::NONE;
  double heapM_meanHeur = -1.0;
  unsigned int heapM_biggest_count = 0;
  for (const std::pair<ConeUpdate::Type, std::pair<unsigned int, double>> &heapM_vote : heapM_votation) {
    if (heapM_vote.first != ConeUpdate::NONE) {
      if (heapM_biggest_count < heapM_vote.second.first or (heapM_biggest_count == heapM_vote.second.first and heapM_meanHeur < heapM_vote.second.second/heapM_vote.second.first)) {
        heapM_biggest_count = heapM_vote.second.first;
        heapM_meanHeur = heapM_vote.second.second/heapM_biggest_count;
        heapM_biggestType = heapM_vote.first;
      }
    }
  }

  // HERE IS THE UPDATE
  // metadata_.valid = true;
  //metadata_.type = ConeUpdate::NONE;
  // if (heap_isMatched) {
  //   if (heapM_meanHeur > 0.007) metadata_.type = heapM_biggestType;
  // }
  // else if (Cone::bothCams and heap_meanHeur > 1/params.dist_cp_to_false_positives) {
  //   metadata_.valid = false;
  // }
  if (heapM_biggestType != ConeUpdate::NONE) {
    metadata_.type = heapM_biggestType;
  }
  // if (!heap_isMatched and Cone::bothCams and heap_meanHeur > 1/params.dist_cp_to_false_positives) {
  //   metadata_.type = ConeUpdate::NONE;
  // }
}

/**
 * PUBLIC METHODS
 */

void Cone::update(const ConeUpdate &coneUpdate) {
  /* ----------------------------- Update heap_ ----------------------------- */
  double heuristic = getHeuristic(coneUpdate, 0.0);
  if (heap_.size() >= params.heap_size) {
    // Find the element with min heuristic in the vector
    // and replace it if the heuristic is bigger.
    auto min_it = std::min_element(heap_.begin(), heap_.end(), Consideration::comparer);

    // Replace the consideration
    if (heuristic > min_it->heuristic) {
      *min_it = Consideration(coneUpdate.type != ConeUpdate::NONE, heuristic);
    }
  } else {
    heap_.emplace_back(coneUpdate.type != ConeUpdate::NONE, heuristic);
  }

  /* ----------------------------- Update HeapM_ ---------------------------- */
  if (coneUpdate.type != ConeUpdate::NONE) {
    double coneHeuristic = getHeuristicM(coneUpdate, 0.0);
    if (heapM_.size() >= params.heap_size) {
      // Find the element with min heuristic in the vector
      // and replace it if the heuristic is bigger.
      auto min_it = std::min_element(heapM_.begin(), heapM_.end(), ConsiderationM::comparer);

      // Replace the consideration
      if (coneHeuristic > min_it->heuristic) {
        *min_it = ConsiderationM(coneUpdate.type, coneHeuristic);
      }
    } else {
      heapM_.emplace_back(coneUpdate.type, coneHeuristic);
    }
  }

  // Update metadata_
  updateMetadata();
}

void Cone::updateObs(const Observation &obs, const double &distSqToPosition) {
  this->obs = std::make_shared<Observation>(obs);
  this->obs->id = id;
}

void Cone::updateLocal(const Eigen::Affine3d &carTf) {
  obs->updateLocal(carTf);
}

/* Getters */

const Point &Cone::position_global() const {
  return obs->centroid_global;
}

const bool &Cone::valid() const {
  return metadata_.valid;
}

as_msgs::Cone Cone::getASCone() const {
  as_msgs::Cone res;
  res.id = static_cast<uint32_t>(id);
  res.position_global = position_global().gmPoint();
  res.position_baseLink = obs->temp.centroid_local.gmPoint();
  res.type = static_cast<uint8_t>(metadata_.type);
  res.confidence = metadata_.confidence;
  return res;
}