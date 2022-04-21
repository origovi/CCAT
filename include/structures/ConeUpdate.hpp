#ifndef STRUCTURES_CONEUPDATE_HPP
#define STRUCTURES_CONEUPDATE_HPP

#include <algorithm>
#include <memory>

#include "as_msgs/BB2.h"
#include "structures/Observation.hpp"

#define CONE_DEFAULT_MATCH_DIST 1e10
#define CONE_DEFAULT_DIST_TO_CAM_PLANE 1e10

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

  enum Operation { ADD,
                   DELETE } operation;
  
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  // No matching update
  ConeUpdate(const size_t &id);

  // Matched update
  ConeUpdate(const size_t &id, const uint8_t &bbType, const double &matchingDist, const double &distToCameraPlane);

  const size_t id;

  const double matchingDist;

  const double distToCameraPlane;

  /**
   * PUBLIC METHODS
   */
  explicit operator bool() const;
  void setTypeFromAsMsgs(const uint8_t &bbType);
  uint8_t typeToAsMsgs() const;
};

#endif  // STRUCTURES_CONEUPDATE_HPP