#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <string>
#include <Eigen/Eigen>

struct Params {
  struct Common {
    int frequency;
    struct {
      struct {
        std::string observations;
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
      } input;
      struct {
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
};

#endif