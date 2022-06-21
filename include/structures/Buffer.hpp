/**
 * @file Buffer.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Buffer class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef STRUCTURES_BUFFER_HPP
#define STRUCTURES_BUFFER_HPP

#include <ros/ros.h>

#include <deque>
#include <stdexcept>

namespace BuffF {
inline ros::Duration abs(const ros::Duration &d) { return ros::Duration().fromNSec(std::abs(d.toNSec())); }
}  // namespace BuffF

/**
 * @brief A class that represents a buffer, implemented with a deque and able to
 * restrict its size with a delta of time.
 * 
 * @tparam BufferedType
 */
template <typename BufferedType>
class Buffer : private std::deque<std::pair<BufferedType, ros::Time>> {
 private:
  using Elem = std::pair<BufferedType, ros::Time>;
  using Parent = std::deque<Elem>;

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief Represents the maximum delta of time that the Buffer can have.
   * If the difference in time of the latest and the oldest is bigger, the older
   * objects will be removed.
   */
  const ros::Duration tempMem_;

 public:
  Buffer(const float &tempMem) : tempMem_(tempMem) {}

  bool empty() const { return Parent::empty(); }

  size_t size() const { return Parent::size(); }

  const ros::Time &oldestStamp() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return Parent::front().second;
  }

  const Elem &oldestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::front();
  }

  const ros::Time &newestStamp() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return Parent::back().second;
  }

  const Elem &newestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::back();
  }

  ros::Duration avgDelay() const {
    if (size() < 2) return ros::Duration();
    return ros::Duration().fromNSec((newestStamp() - oldestStamp()).toNSec() / (size() - 1));
  }

  double avgFreq() const {
    if (size() < 2) return 0.0;
    return 1.0 / avgDelay().toSec();
  }

  template <typename BufferedType2>
  bool isSynchWith(const Buffer<BufferedType2> &buffer, BufferedType &thisSyncElem, BufferedType2 &syncElem) const {
    ros::Duration diff;

    Elem thisElem;
    std::pair<BufferedType2, ros::Time> elem;
    // this has a newer element
    if (this->newestStamp() >= buffer.newestStamp()) {
      elem = buffer.newestElem();
      thisElem = this->elemClosestTo(elem.second);
      diff = BuffF::abs(thisElem.second - elem.second);
    }
    // buffer (parameter) has a newer element
    else {
      thisElem = this->newestElem();
      elem = buffer.elemClosestTo(thisElem.second);
      diff = BuffF::abs(elem.second - thisElem.second);
    }

    if (this->avgFreq() > buffer.avgFreq()) {
      if (diff < this->avgDelay() * 0.5) {
        thisSyncElem = thisElem.first;
        syncElem = elem.first;
        return true;
      }
      return false;
    } else {
      if (diff < buffer.avgDelay() * 0.5) {
        thisSyncElem = thisElem.first;
        syncElem = elem.first;
        return true;
      }
      return false;
    }
  }

  void add(const BufferedType &element, const ros::Time &stamp = ros::Time::now()) {
    Parent::emplace_back(element, stamp);

    // Readjust
    while (!empty() and BuffF::abs(newestStamp() - oldestStamp()) > tempMem_) {
      Parent::pop_front();
    }
  }

  const Elem &elemJustBefore(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (oldestStamp() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = Parent::crbegin(); it != Parent::crend(); it++) {
      if (it->second < stamp) {
        return *it;
      }
    }
  }

  const Elem &elemJustAfter(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (newestStamp() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = Parent::cbegin(); it != Parent::cend(); it++) {
      if (it->second > stamp) {
        return *it;
      }
    }
  }

  const Elem &elemClosestTo(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }

    auto closestElem = Parent::crbegin();
    ros::Duration closestDiff = BuffF::abs(closestElem->second - stamp);

    for (auto it = Parent::crbegin(); it != Parent::crend(); it++) {
      ros::Duration diff = BuffF::abs(it->second - stamp);
      if (diff <= closestDiff) {
        closestDiff = diff;
        closestElem = it;
      } else {
        break;
      }
    }
    return *closestElem;
  }
};

#endif  // STRUCTURES_BUFFER_HPP