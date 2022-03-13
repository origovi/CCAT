#ifndef CONE_HPP
#define CONE_HPP

#include "Observation.hpp"

class Cone : public Observation {
 public:
  Cone();
  Cone(const Observation &observation);

  ~Cone();

  enum Type { None, Blue, Yellow, SmallOrange, BigOrange } type;

  /* Getters */
};

#endif  // CONE_HPP