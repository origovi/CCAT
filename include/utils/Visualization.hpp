/**
 * @file Visualization.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Visualization class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef UTILS_VISUALIZATION_HPP
#define UTILS_VISUALIZATION_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>
#include <stdexcept>
#include <string>

/**
 * @brief Abstract-static class made to make it easier to publish information
 * to topics. It creates dynamically ROS::Publisher(s) for the following
 * datatypes:
 * - sensor_msgs::PointCloud2
 * - visualization_msgs::MarkerArray
 * - visualization_msgs::Marker
 * - visualization_msgs::MarkerArray
 * - image_transport::ImageTransport
 */
class Visualization {
 private:
  template <typename T>
  struct RosPub : ros::Publisher {
    RosPub(const std::string &topic, const size_t &defQueueSize) : ros::Publisher(nh_->advertise<T>(topic, defQueueSize)) {}
  };

  struct ImgPub : image_transport::Publisher {
    ImgPub(const std::string &topic, const size_t &defQueueSize) : image_transport::Publisher(image_transport::ImageTransport(*nh_).advertise(topic, defQueueSize)) {}
  };

  template <typename T>
  struct DB : std::map<std::string, T> {
    T &get(const std::string &topic, const size_t &defQueueSize = 1) {
      if (nh_ == nullptr)
        throw std::runtime_error("Call init method before publishing a message through Visualization");
      auto it = this->find(topic);
      if (it == this->end()) {
        auto itemIt = this->emplace(std::piecewise_construct, std::forward_as_tuple(topic), std::forward_as_tuple(topic, defQueueSize));
        return itemIt.first->second;
      } else {
        return it->second;
      }
    }
  };

  static ros::NodeHandle *nh_;

  static DB<RosPub<sensor_msgs::PointCloud2>> pubPcl_;
  static DB<RosPub<visualization_msgs::MarkerArray>> pubMA_;
  static DB<RosPub<visualization_msgs::Marker>> pubM_;
  static DB<ImgPub> pubImg_;

 protected:
  Visualization();

  ros::Publisher &pubPcl(const std::string &topic, const size_t &defQueueSize = 1);
  ros::Publisher &pubMA(const std::string &topic, const size_t &defQueueSize = 1);
  ros::Publisher &pubM(const std::string &topic, const size_t &defQueueSize = 1);
  image_transport::Publisher &pubImg(const std::string &topic, const size_t &defQueueSize = 1);

 public:
  static void init(ros::NodeHandle *const nh);
};

#endif  // UTILS_VISUALIZATION_HPP