#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

struct Params {
  struct Common {
    int frequency;
    struct {
      struct {
        std::string observations;
        std::string odometry;
        std::string map;
      } input;
      struct {
      } output;
    } topics;
  } common;
  struct Preproc {
    float cluster_dist;
    struct {
      struct {
        std::string observations;
        std::string map;
      } input;
      struct {
      } output;
    } topics;
  } preproc;
  struct Matcher {
    struct {
      std::vector<double> translation;
      std::vector<double> euler_angles;
    } extrinsics_right, extrinsics_left;
    std::vector<double> intrinsics_right, intrinsics_left;
    struct {
      struct {
        std::string right_detections, left_detections;
      } input;
      struct {
        std::string projected_right, projected_left;
      } output;
    } topics;
  } matcher;
  struct Tracker {
    bool x;
    struct {
      struct {
      } input;
      struct {
      } output;
    } topics;
  } tracker;
  Params(const ros::NodeHandle &nh);
};

#endif