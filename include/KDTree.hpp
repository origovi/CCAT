/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 * 
 * It is a reimplementation of the C code using C++. It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 * 
 */

#ifndef KDTREE_HPP
#define KDTREE_HPP

#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "Point.hpp"

using indexArr = std::vector<size_t>;
using pointIndex = typename std::pair<Point, size_t>;
#endif

class KDNode {
 public:
  using KDNodePtr = std::shared_ptr<KDNode>;
  size_t index;
  Point x;
  KDNodePtr left;
  KDNodePtr right;

  // initializer
  KDNode();
  KDNode(const Point &, const size_t &, const KDNodePtr &,
         const KDNodePtr &);
  KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
  ~KDNode();

  // getter
  double coord(const size_t &);

  // conversions
  explicit operator bool();
  explicit operator Point();
  explicit operator size_t();
  explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr<KDNode>;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline double dist2(const Point &, const Point &);
inline double dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline double dist(const Point &, const Point &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
 public:
  size_t idx;
  explicit comparer(size_t idx_);
  inline bool compare_idx(
      const std::pair<Point, size_t> &,  //
      const std::pair<Point, size_t> &   //
  );
};

using pointIndexArr = typename std::vector<pointIndex>;

inline void sort_on_idx(const pointIndexArr::iterator &,  //
                        const pointIndexArr::iterator &,  //
                        size_t idx);

using pointVec = std::vector<Point>;

class KDTree {
  KDNodePtr root;
  KDNodePtr leaf;

  KDNodePtr make_tree(const pointIndexArr::iterator &begin,  //
                      const pointIndexArr::iterator &end,    //
                      const size_t &length,                  //
                      const size_t &level                    //
  );

 public:
  KDTree() = default;
  explicit KDTree(pointVec point_array);

 private:
  KDNodePtr nearest_(           //
      const KDNodePtr &branch,  //
      const Point &pt,          //
      const size_t &level,      //
      const KDNodePtr &best,    //
      const double &best_dist   //
  );

  // default caller
  KDNodePtr nearest_(const Point &pt);

 public:
  Point nearest_point(const Point &pt);
  size_t nearest_index(const Point &pt);
  pointIndex nearest_pointIndex(const Point &pt);

 private:
  pointIndexArr neighborhood_(  //
      const KDNodePtr &branch,  //
      const Point &pt,          //
      const double &rad,        //
      const size_t &level       //
  ) const;

 public:
  pointIndexArr neighborhood(  //
      const Point &pt,         //
      const double &rad) const;

  pointVec neighborhood_points(  //
      const Point &pt,           //
      const double &rad);

  indexArr neighborhood_indices(  //
      const Point &pt,            //
      const double &rad) const;
};
