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
        std::string left_bbs, right_bbs;
      } input;
      struct {
      } output;
    } topics;
  } common;
  struct Preproc {
    float cluster_dist;
    struct {
      struct {
      } input;
      struct {
      } output;
    } topics;
  } preproc;
  struct Matcher {
    int image_width, image_height;
    struct {
      std::vector<double> translation;
      std::vector<double> euler_angles;
    } extrinsics;
    std::vector<double> intrinsics;
    struct {
      struct {
      } input;
      struct {
        std::string projected;
      } output;
    } topics;
  } matcherL, matcherR;
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