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
  /* -------------------------- Public Constructor -------------------------- */

  Buffer(const float &tempMem) : tempMem_(tempMem) {}

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Whether or not the container does not have data.
   * 
   * @return true 
   * @return false 
   */
  bool empty() const { return Parent::empty(); }

  /**
   * @brief Returns the number of objects in the container.
   * 
   * @return size_t 
   */
  size_t size() const { return Parent::size(); }

  /**
   * @brief Returns the stamp of the oldest element.
   * 
   * @return const ros::Time& 
   */
  const ros::Time &oldestStamp() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return Parent::front().second;
  }

  /**
   * @brief Returns the oldest element.
   * 
   * @return const Elem& 
   */
  const Elem &oldestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::front();
  }

  /**
   * @brief Returns the timestamp of the newest element.
   * 
   * @return const ros::Time& 
   */
  const ros::Time &newestStamp() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return Parent::back().second;
  }

  /**
   * @brief Returns the newest element.
   * 
   * @return const Elem& 
   */
  const Elem &newestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::back();
  }

  /**
   * @brief Makes an average of the delay between the data stored in the
   * container.
   * 
   * @return ros::Duration 
   */
  ros::Duration avgDelay() const {
    if (size() < 2) return ros::Duration();
    return ros::Duration().fromNSec((newestStamp() - oldestStamp()).toNSec() / (size() - 1));
  }

  /**
   * @brief Makes an average of the frequency between the data stored in the
   * container.
   * 
   * @return double 
   */
  double avgFreq() const {
    if (size() < 2) return 0.0;
    return 1.0 / avgDelay().toSec();
  }

  /**
   * @brief Computes whether or not a specific Buffer is in synch with \a this
   * Buffer, if so, \a thisSyncElem and \a syncElem will be the synched
   * elements.
   * 
   * @tparam BufferedType2 
   * @param buffer 
   * @param thisSyncElem 
   * @param syncElem 
   * @return true 
   * @return false 
   */
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

  /**
   * @brief Emplaces a new element into the Buffer with timestamp \a stamp.
   * 
   * @param element 
   * @param stamp 
   */
  void add(const BufferedType &element, const ros::Time &stamp = ros::Time::now()) {
    Parent::emplace_back(element, stamp);

    // Readjust
    while (!empty() and BuffF::abs(newestStamp() - oldestStamp()) > tempMem_) {
      Parent::pop_front();
    }
  }

  /**
   * @brief Returns the element that has a timestamp just before \a stamp. 
   * 
   * @param stamp 
   * @return const Elem& 
   */
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

  /**
   * @brief Returns the element that has a timestamp just after \a stamp. 
   * 
   * @param stamp 
   * @return const Elem& 
   */
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

  /**
   * @brief Returns the element that has a timestamp closest to \a stamp. 
   * 
   * @param stamp 
   * @return const Elem& 
   */
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

  /**
   * @brief Finds the elements just before and after \a t
   * 
   * @param[in] t is the reference stamp
   * @param[out] e1 is the element with stamp just before \a t
   * @param[out] e2 is the element with stamp just after \a t
   */
  bool hasOlderAndNewer(const ros::Time &t, BufferedType& e1, BufferedType &e2) const {
    bool older = false;
    bool newer = false;
    for (auto it = Parent::crbegin(); it != Parent::crend() and (!older or !newer); it++) {
      if (it->second > t) {
        newer = true;
        e2 = it->first;
      }
      else if (!newer) return false;
      else if (it->second < t) {
        older = true;
        e1 = it->first;
      }
    }
    return older and newer;
  }

  /**
   * @brief Synchronizes \a this and \a buffer and returns the latest element in
   * \a this that has an element before and after in \a buffer
   * 
   * @param[in] buffer is a Buffer to which data will be synchronized
   * @param[out] data latest element in \a this that has an element before and after in \a buffer
   * @param[out] lowerBound element just before \a data in \a buffer
   * @param[out] upperBound element just after \a data in \a buffer
   * @returns whether or not the synchronization was successful
   */
  template <typename BufferedType2>
  bool dataWithBoundsInBuffParam(const Buffer<BufferedType2> &buffer, BufferedType &data, BufferedType2 &lowerBound, BufferedType2 &upperBound) const {
    for (auto it = Parent::crbegin(); it != Parent::crend(); it++) {
      if (buffer.hasOlderAndNewer(it->second, lowerBound, upperBound)) {
        data = it->first;
        return true;
      }
    }
    return false;
  }
};

#endif  // STRUCTURES_BUFFER_HPP