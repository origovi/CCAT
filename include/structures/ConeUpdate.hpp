/**
 * @file ConeUpdate.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the ConeUpdate class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef STRUCTURES_CONEUPDATE_HPP
#define STRUCTURES_CONEUPDATE_HPP

#include <algorithm>
#include <memory>

#include "as_msgs/BB2.h"
#include "structures/Observation.hpp"

#define CONE_DEFAULT_MATCH_DIST 1e10
#define CONE_DEFAULT_DIST_TO_CLOSEST_MATCH 1e10
#define CONE_DEFAULT_PROB_BB 1.0

/**
 * @brief Represents an update to a Cone.
 * 
 */
class ConeUpdate {
 public:
  /* --------------------------- Public Attributes -------------------------- */
  
  enum Type { YELLOW,
              BLUE,
              SMALLORANGE,
              BIGORANGE,
              NONE } type;

  /* -------------------------- Public Constructors ------------------------- */

  // No matching update
  ConeUpdate(const size_t &id, const double &distToCameraPlane, const double &distToClosestMatch = CONE_DEFAULT_DIST_TO_CLOSEST_MATCH);

  // Matched update
  ConeUpdate(const size_t &id, const uint8_t &bbType, const float &probBB, const double &matchingDist, const double &distToCameraPlane);

  /**
   * @brief Id of the Cone it refers to.
   */
  const size_t id;

  /**
   * @brief (If applies) the distance of matching of the cone (in pixels).
   */
  const double matchingDist;

  /**
   * @brief (If applies) the distance to the camera plane of the cone.
   */
  const double distToCameraPlane;

  /**
   * @brief (If applies) the distance to the closest cone that is matched in
   * the same iteration.
   */
  const double distToClosestMatch;

  /**
   * @brief (If applies) the confidence of the BB it is matched to.
   */
  const float probBB;

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Returns whether or not this cone has a type not equal to UNK.
   * 
   * @return true 
   * @return false 
   */
  explicit operator bool() const;

  /**
   * @brief Set the Type From As Msgs object.
   * 
   * @param bbType 
   */
  void setTypeFromAsMsgs(const uint8_t &bbType);

  /**
   * @brief Converts the cone type to as_msgs.
   * 
   * @return uint8_t 
   */
  uint8_t typeToAsMsgs() const;
};

#endif  // STRUCTURES_CONEUPDATE_HPP