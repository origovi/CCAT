#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>
#include <string>

class Visualization {
 private:
  template <typename T>
  struct RosPub : ros::Publisher {
    RosPub(ros::NodeHandle &nh, const std::string &topic, const size_t &defQueueSize) : ros::Publisher(nh.advertise<T>(topic, defQueueSize)) {}
  };

  struct ImgPub : image_transport::Publisher {
    ImgPub(ros::NodeHandle &nh, const std::string &topic, const size_t &defQueueSize) : image_transport::Publisher(image_transport::ImageTransport(nh).advertise(topic, defQueueSize)) {}
  };

  template <typename T>
  struct DB : std::map<std::string, T> {
    T &get(ros::NodeHandle &nh, const std::string &topic, const size_t &defQueueSize = 1) {
      auto it = this->find(topic);
      if (it == this->end()) {
        auto itemIt = this->emplace(std::piecewise_construct, std::forward_as_tuple(topic), std::forward_as_tuple(nh, topic, defQueueSize));
        return itemIt.first->second;
      } else {
        return it->second;
      }
    }
  };

  ros::NodeHandle *const nh_;

  DB<RosPub<sensor_msgs::PointCloud2>> pubPcl_;
  DB<RosPub<visualization_msgs::MarkerArray>> pubMA_;
  DB<RosPub<visualization_msgs::Marker>> pubM_;
  DB<ImgPub> pubImg_;


 protected:
  Visualization(ros::NodeHandle *const nh);
  
  ros::Publisher &pubPcl(const std::string &topic, const size_t &defQueueSize = 1);
  ros::Publisher &pubMA(const std::string &topic, const size_t &defQueueSize = 1);
  ros::Publisher &pubM(const std::string &topic, const size_t &defQueueSize = 1);
  image_transport::Publisher &pubImg(const std::string &topic, const size_t &defQueueSize = 1);
};

#endif