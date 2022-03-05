#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <string>

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
      float cone_size_width;
      float cone_size_height;
      float safety_factor;
    } reconstruct;
  } matcher;
  struct Tracker {
    bool x;
  } tracker;
};

#endif