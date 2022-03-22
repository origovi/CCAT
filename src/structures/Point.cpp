#include "structures/Point.hpp"

/**
 * CONSTRUCTORS
 */
Point::Point() : x(0.0), y(0.0), z(0.0) {}
Point::Point(const double &x, const double &y, const double &z) : x(x), y(y), z(z) {}

template <typename T>
Point::Point(const T &point) : x(point.x), y(point.y), z(point.z) {}
template Point::Point<geometry_msgs::Point>(const geometry_msgs::Point &);
template Point::Point<pcl::PointXYZI>(const pcl::PointXYZI &);

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
Point Point::operator+(const Point &p) const { return Point(x + p.x, y + p.y, z + p.z); }

Point Point::operator-(const Point &p) const { return Point(x - p.x, y - p.y, z - p.z); }

template <typename T>
Point Point::operator*(const T &num) const {
  return Point(x * num, y * num, z * num);
}
template Point Point::operator*<int>(const int &) const;
template Point Point::operator*<float>(const float &) const;
template Point Point::operator*<double>(const double &) const;
template Point Point::operator*<size_t>(const size_t &) const;

template <typename T>
Point Point::operator/(const T &num) const {
  return Point(x / num, y / num, z / num);
}
template Point Point::operator/<int>(const int &) const;
template Point Point::operator/<float>(const float &) const;
template Point Point::operator/<double>(const double &) const;
template Point Point::operator/<size_t>(const size_t &) const;

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

template <typename T>
Point &Point::operator*=(const T &num) {
  x *= num;
  y *= num;
  z *= num;
  return *this;
}
template Point &Point::operator*=<int>(const int &);
template Point &Point::operator*=<float>(const float &);
template Point &Point::operator*=<double>(const double &);
template Point &Point::operator*=<size_t>(const size_t &);

template <typename T>
Point &Point::operator/=(const T &num) {
  x /= num;
  y /= num;
  z /= num;
  return *this;
}
template Point &Point::operator/=<int>(const int &);
template Point &Point::operator/=<float>(const float &);
template Point &Point::operator/=<double>(const double &);
template Point &Point::operator/=<size_t>(const size_t &);

std::ostream &operator<<(std::ostream &os, const Point &p) {
  return os << "Point(" << p.x << ", " << p.y << ", " << p.z << ")\n";
}

void Point::transform(const Eigen::Affine3d &tf) {
  Eigen::Vector3d product = tf*Eigen::Vector3d(x, y, z);
  x = product.x();
  y = product.y();
  z = product.z();
}

Point Point::transformed(const Eigen::Affine3d &tf) {
  Eigen::Vector3d product = tf*Eigen::Vector3d(x, y, z);
  return Point(product.x(), product.y(), product.z());
}

Eigen::Vector3f Point::vec3f() const {
  return Eigen::Vector3f(float(x), float(y), float(z));
}

geometry_msgs::Point Point::gmPoint() const {
  geometry_msgs::Point res;
  res.x = x;
  res.y = y;
  res.z = z;
  return res;
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