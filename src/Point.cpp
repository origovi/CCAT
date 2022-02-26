#include "Point.hpp"

/**
 * CONSTRUCTORS
 */
Point::Point() : x(0.0), y(0.0), z(0.0) {}
Point::Point(const double &x, const double &y, const double &z) : x(x), y(y), z(z) {}
Point::Point(const as_msgs::Observation &observation) : x(observation.x), y(observation.y), z(observation.z) {}

/**
 * DESTRUCTORS
 */
Point::~Point() {}

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */
Point &Point::operator+=(const Point &p) {
  x += p.x;
  y += p.y;
  z += p.z;
  return *this;
}

Point &Point::operator-=(const Point &p) {
  x -= p.x;
  y -= p.y;
  z -= p.z;
  return *this;
}

Point &Point::operator/=(const int &num) {
  x /= num;
  y /= num;
  z /= num;
  return *this;
}

/* Getters */
const double &Point::at(const size_t &ind) const {
  switch (ind) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      return z;
  }
}

size_t Point::size() const { return 3; }