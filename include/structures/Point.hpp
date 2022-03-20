#ifndef POINT_HPP
#define POINT_HPP

#include <as_msgs/Observation.h>

#include <cmath>
#include <pcl/point_types.h>
#include <iostream>
#include <Eigen/Geometry>

class Point {
 private:
  /**
   * PRIVATE ATTRIBUTES
   */

  /**
   * PRIVATE METHODS
   */
 public:
  /**
   * CONSTRUCTORS AND DESTRUCTOR
   */

  Point();
  Point(const double &x, const double &y, const double &z);
  template<typename T>
  Point(const T &point);
  ~Point();

  /**
   * PUBLIC ATTRIBUTES
   */

  double x, y, z;

  /**
   * PUBLIC METHODS
   */
  
  Point operator+(const Point &p) const;
  Point operator-(const Point &p) const;
  
  template<typename T>
  Point operator*(const T &num) const;
  
  template<typename T>
  Point operator/(const T &num) const;
  
  Point &operator+=(const Point &p);
  Point &operator-=(const Point &p);
  
  template<typename T>
  Point &operator*=(const T &num);

  template<typename T>
  Point &operator/=(const T &num);

  static inline double distSq(const Point &p1, const Point &p2 = Point()) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
  }

  static inline double dist(const Point &p1, const Point &p2 = Point()) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
  }

  friend std::ostream &operator<<(std::ostream &os, const Point &p);

  void transform(const Eigen::Affine3d &tf);
  Eigen::Vector3f vec3f() const;
  geometry_msgs::Point gmPoint() const;

  
  /* Getters */
  const double &at(const size_t &ind) const;
  size_t size() const;
};

#endif  // POINT_HPP