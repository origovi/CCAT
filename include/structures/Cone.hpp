#ifndef CONE_HPP
#define CONE_HPP

#include <memory>

#include "as_msgs/BB2.h"
#include "structures/Observation.hpp"

class Cone : public Observation {
 public:
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  Cone();
  Cone(const Observation &observation);
  ~Cone();

  /**
   * PUBLIC ATTRIBUTES
   */

  enum Type { None, Blue, Yellow, SmallOrange, BigOrange } type;

  /**
   * PUBLIC METHODS
   */

  void setTypeFromAsMsgs(const uint8_t &type);
};

#endif  // CONE_HPP