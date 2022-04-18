#ifndef STRUCTURES_BUFFER_HPP
#define STRUCTURES_BUFFER_HPP

#include <ros/ros.h>

#include <deque>
#include <stdexcept>

namespace BuffF {
inline ros::Duration abs(const ros::Duration &d) { return ros::Duration().fromNSec(std::abs(d.toNSec())); }
}  // namespace BuffF

template <typename BufferedType>
class Buffer : private std::deque<std::pair<BufferedType, ros::Time>> {
 private:
  using Parent = std::deque<std::pair<BufferedType, ros::Time>>;
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

  const BufferedType &oldestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::front().first;
  }

  const ros::Time &newestStamp() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get stamp");
    }
    return Parent::back().second;
  }

  const BufferedType &newestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::back().first;
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
  bool isSynchWith(const Buffer<BufferedType2> &buffer) const {
    ros::Duration diff = BuffF::abs(newestStamp() - buffer.newestStamp());
    if (avgFreq() > buffer.avgFreq()) {
      return diff < avgDelay() * 0.5;
    } else {
      return diff < buffer.avgDelay() * 0.5;
    }
  }

  void add(const BufferedType &element, const ros::Time &stamp = ros::Time::now()) {
    Parent::emplace_back(element, stamp);

    // Readjust
    while (newestStamp() - oldestStamp() > tempMem_) {
      Parent::pop_front();
    }
  }

  const BufferedType &elemJustBefore(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (oldestStamp() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = Parent::crbegin(); it != Parent::crend(); it++) {
      if (it->second < stamp) {
        return it->first;
      }
    }
  }

  const BufferedType &elemJustAfter(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }
    if (newestStamp() >= stamp) {
      throw std::runtime_error("There is no element before stamp in Buffer");
    }

    for (auto it = Parent::cbegin(); it != Parent::cend(); it++) {
      if (it->second > stamp) {
        return it->first;
      }
    }
  }

  const BufferedType &elemClosestTo(const ros::Time &stamp) const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get any element");
    }

    ros::Duration closestDiff = BuffF::abs(Parent::back().second - stamp);
    const BufferedType &closestElem = Parent::back().first;

    for (auto it = Parent::crbegin(); it != Parent::crend(); it++) {
      ros::Duration diff = BuffF::abs(it->second - stamp);
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