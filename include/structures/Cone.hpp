#ifndef CONE_HPP
#define CONE_HPP

#include <memory>

#include "as_msgs/BB2.h"
#include "structures/Observation.hpp"
#include <algorithm>

#define CONE_DEFAULT_MATCH_DIST 1e10

class Cone {
 public:
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  Cone();
  Cone(const Observation::Ptr &_observation, const double &_matchingScore = CONE_DEFAULT_MATCH_DIST);
  ~Cone();

  /**
   * PUBLIC ATTRIBUTES
   */

  enum Type { Yellow,
              Blue,
              SmallOrange,
              BigOrange,
              None } type;

  enum Operation { ADD,
                   DELETE } operation;

  // considerar treure la observacio
  Observation::Ptr observation;
  double matchingDist;

  /**
   * PUBLIC METHODS
   */
  explicit operator bool() const;
  void setTypeFromAsMsgs(const uint8_t &type);
  uint8_t typeToAsMsgs() const;
};

#endif  // CONE_HPP