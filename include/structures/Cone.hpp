/**
 * @file Cone.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Cone class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef STRUCTURES_CONE_HPP
#define STRUCTURES_CONE_HPP

#include <Eigen/Geometry>
#include <algorithm>

#include <as_msgs/Cone.h>
#include "structures/ConeUpdate.hpp"
#include "structures/Observation.hpp"
#include "structures/Point.hpp"
#include "structures/Params.hpp"

/**
 * @brief Represents a cone, here is where the statistical model is applied.
 * All decisions in respect a cone type and whether or not a cone is valid
 * or false positive.
 */
class Cone {
 private:
  /* ---------------------------- Private Structs --------------------------- */

  struct ConsiderationM {
    ConeUpdate::Type type;
    double heuristic;
    ConsiderationM(const ConeUpdate::Type &type, const double &heuristic) : type(type), heuristic(heuristic) {}
    static bool comparer(const ConsiderationM &c1, const ConsiderationM &c2) { return c1.heuristic < c2.heuristic; }
  };

  struct Consideration {
    bool isMatched;
    double heuristic;
    Consideration(const bool &isMatched, const double &heuristic) : isMatched(isMatched), heuristic(heuristic) {}
    static bool comparer(const Consideration &c1, const Consideration &c2) { return c1.heuristic < c2.heuristic; }
  };

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief Is the vector of matched considerations, it will hold the \a n best
   * matches.
   */
  std::vector<ConsiderationM> heapM_;

  /**
   * @brief Is the vector of non-matched considerations, it will hold all the
   * observations. From this object, we can decide if a cone is valid or not.
   */
  std::vector<Consideration> heap_;
  
  /**
   * @brief All data regarding the cone's final decisions.
   */
  struct {
    ConeUpdate::Type type;
    float confidence;
    bool valid;
  } metadata_;

  /**
   * @brief Computes the heuristic of an specific match.
   * 
   * @param coneUpdate 
   * @param distSqToOldPos 
   * @return double 
   */
  static double getHeuristicM(const ConeUpdate &coneUpdate, const double &distSqToOldPos);
  
  /**
   * @brief Computes the heuristic for all observations (matched and
   * non-matched).
   * 
   * @param coneUpdate 
   * @param distSqToOldPos 
   * @return double 
   */
  static double getHeuristic(const ConeUpdate &coneUpdate, const double &distSqToOldPos);
  
  /**
   * @brief Updates the \a metadata_ object with latest decisions.
   * 
   */
  void updateMetadata();

 public:

  /* -------------------------- Public Constructor -------------------------- */

  Cone(const Observation &obs, const size_t &id);

  /* --------------------------- Public Attributes -------------------------- */

  /**
   * @brief The parameters needed for the application of the statistical model.
   */
  static Params::Tracker::Cone params;

  static bool bothCams;

  /**
   * @brief A shared_ptr to the Observation that represents this cone.
   */
  Observation::Ptr obs;

  /**
   * @brief The unique id of this cone.
   */
  const size_t id;

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Updates all cone information and decisions with a new ConeUpdate
   * object direct from the Merger.
   * 
   * @param coneUpdate 
   */
  void update(const ConeUpdate &coneUpdate);

  /**
   * @brief Updates the Observation information (centroid, point cloud, ...).
   * 
   * @param obs 
   * @param distSqToPosition 
   */
  void updateObs(const Observation &obs, const double &distSqToPosition);

  /**
   * @brief Updates the position of this cone. This is necessary because the
   * car is constantly moving. Local coordinates must be updated.
   * 
   * @param carTf 
   */
  void updateLocal(const Eigen::Affine3d &carTf);

  /**
   * @brief Returns the global position of the cone.
   * 
   * @return const Point& 
   */
  const Point &position_global() const;

  /**
   * @brief Returns whether of not a cone is valid or a false positive.
   * 
   * @return true 
   * @return false 
   */
  const bool &valid() const;

  /**
   * @brief Returns the cone is as_msgs format.
   * 
   * @return as_msgs::Cone 
   */
  as_msgs::Cone getASCone() const;
};

#endif  // STRUCTURES_CONE_HPP