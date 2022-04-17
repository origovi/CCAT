#ifndef STRUCTURES_BUFFER_HPP
#define STRUCTURES_BUFFER_HPP

#include <ros/ros.h>

#include <deque>
#include <stdexcept>

template <typename BufferedType>
class Buffer : private std::deque<std::pair<BufferedType, ros::Time>> {
 private:
  const ros::Duration tempMem_;

 public:
  Buffer(const float &tempMem) : tempMem_(tempMem) {}

  bool empty() const { return this->empty(); }

  size_t size() const { return this->size(); }

  const ros::Time &oldest() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return this->front().second;
  }

  const ros::Time &newest() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return this->back().second;
  }

  void add(const BufferedType &element, const ros::Time &stamp = ros::Time::now()) {
    this->emplace_back(element, stamp);

    // Readjust
    while (newest() - oldest() > tempMem_) {
      this->pop_front();
    }
  }

  const BufferedType &elemJustBefore(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (oldest() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = this->crbegin(); it != this->crend(); it++) {
      if (it->second < stamp) {
        return it->first;
      }
    }
  }

  const BufferedType &elemJustAfter(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (newest() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = this->cbegin(); it != this->cend(); it++) {
      if (it->second > stamp) {
        return it->first;
      }
    }
  }

  const BufferedType &elemClosestTo(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }

    ros::Duration closestDiff = std::abs(this->back().second - stamp);
    const BufferedType &closestElem = this->back().first;

    for (auto it = this->crbegin(); it != this->crend(); it++) {
      ros::Duration diff = std::abs(it->second - stamp);
      if (diff <= closestDiff) {
        closestDiff = diff;
        closestElem = it->first;
      } else {
        break;
      }
    }
    return closestElem;
  }
};

#endif  // STRUCTURES_BUFFER_HPP