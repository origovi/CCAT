#ifndef REFERENCE_HPP
#define REFERENCE_HPP

#include <cmath>

#include "Point.hpp"

class Reference : public Point {
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
  Reference(const float &x, const float &y, const float &z);
  /**
   * DESTRUCTORS
   */
  ~Reference();
  /**
   * PUBLIC METHODS
   */
  void addDetection();
  /* Getters */
  int getHash() const;
};

#endif  // REFERENCE_HPP