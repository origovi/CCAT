#ifndef OBSERVATION_HPP
#define OBSERVATION_HPP

#include <cmath>

#include "Point.hpp"
#include "as_msgs/Observation.h"

class Observation {
 private:
  /**
   * PRIVATE ATTRIBUTES
   */

  /**
   * PRIVATE METHODS
   */

 public:
  /**
   * CONSTRUCTORS
   */
  Observation();
  Observation(const float &x, const float &y, const float &z, const float &confidence);
  Observation(const as_msgs::Observation &obs);

  /**
   * DESTRUCTORS
   */
  ~Observation();

  /**
   * PUBLIC ATTRIBUTES
   */
  Point p;
  double confidence;

  /**
   * PUBLIC METHODS
   */
  Observation &operator+=(const Observation &o);
  Observation &operator-=(const Observation &o);
  Observation &operator/=(const int &num);

  /* Getters */
  const double &at(const size_t &ind) const;
  size_t size() const;
  void addDetection();

  /* Getters */
  int getHash() const;
};

#endif  // OBSERVATION_HPP