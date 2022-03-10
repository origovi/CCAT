#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <string>
#include <Eigen/Eigen>

struct Params {
  struct Common {
    int frequency;
  } common;
  struct Topics {
    struct {
      std::string observations;
      std::string map;
    } input;
    struct {
    } output;
  } topics;
  struct Preproc {
    float cluster_dist;
  } preproc;
  struct Matcher {
    struct {
      float cone_width;
      float cone_height;
      float safety_factor;
    } reconstruct;
    Eigen::Affine3d tf_right, tf_left;
    struct Intrinsics {
      float fx, fy;
      float cx, cy;
    } intrinsics_right, intrinsics_left;
  } matcher;
  struct Tracker {
    bool x;
  } tracker;
};

#endif