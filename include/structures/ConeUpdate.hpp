#ifndef STRUCTURES_CONEUPDATE_HPP
#define STRUCTURES_CONEUPDATE_HPP

#include <algorithm>
#include <memory>

#include "as_msgs/BB2.h"
#include "structures/Observation.hpp"

#define CONE_DEFAULT_MATCH_DIST 1e10
#define CONE_DEFAULT_DIST_TO_CLOSEST_MATCH 1e10
#define CONE_DEFAULT_PROB_BB 1.0

class ConeUpdate {
 public:

  /**
   * PUBLIC ATTRIBUTES
   */
  enum Type { YELLOW,
              BLUE,
              SMALLORANGE,
              BIGORANGE,
              NONE } type;

  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  // No matching update
  ConeUpdate(const size_t &id, const double &distToCameraPlane, const double &distToClosestMatch = CONE_DEFAULT_DIST_TO_CLOSEST_MATCH);

  // Matched update
  ConeUpdate(const size_t &id, const uint8_t &bbType, const float &probBB, const double &matchingDist, const double &distToCameraPlane);

  const size_t id;

  const double matchingDist;

  const double distToCameraPlane;

  const double distToClosestMatch;

  const float probBB;

  /**
   * PUBLIC METHODS
   */
  explicit operator bool() const;
  void setTypeFromAsMsgs(const uint8_t &bbType);
  uint8_t typeToAsMsgs() const;
};

#endif  // STRUCTURES_CONEUPDATE_HPP