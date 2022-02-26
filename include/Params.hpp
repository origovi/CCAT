#ifndef PARAMS_HPP
#define PARAMS_HPP

struct Params {
  struct Tracker {
    float clusterDist;
  } tracker;
  struct Matcher {
    bool x;
  } matcher;
};

#endif