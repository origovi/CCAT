#ifndef POINT_HPP
#define POINT_HPP

#include <as_msgs/Observation.h>

#include <cmath>

class Point {
 private:
  /**
   * PRIVATE ATTRIBUTES
   */

  /**
   * PRIVATE METHODS
   */
 public:
  double x, y, z;
  /**
   * CONSTRUCTORS
   */
  Point();
  Point(const double &x, const double &y, const double &z);
  Point(const as_msgs::Observation &observation);

  /**
   * DESTRUCTORS
   */
  ~Point();

  /**
   * PUBLIC METHODS
   */
  Point operator+(const Point &p) const;
  Point &operator+=(const Point &p);
  Point &operator-=(const Point &p);
  Point &operator/=(const int &num);

  static inline double distSq(const Point &p1, const Point &p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
  }
  
  /* Getters */
  const double &at(const size_t &ind) const;
  size_t size() const;
};

#endif  // POINT_HPP