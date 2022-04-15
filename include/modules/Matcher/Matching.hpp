#ifndef MODULES_MATCHER_MATCHING_HPP
#define MODULES_MATCHER_MATCHING_HPP

#include <cstddef>

/**
 * @brief Represents a Matching between a bounding box and an Observation.
 * 
 * Usually in a vector of Matching(s), element \a i specifies that
 * Observation \a i is matching to the BB with index = \a bbInd_.
 */
class Matching {
 private:
  /**
  * @brief Whether or not this match is valid. Initialized to false.
  */
  bool valid_ = false;

  /**
   * @brief The BB index this Observation is matched to.
   */
  size_t bbInd_;

  /**
   * @brief The matching distance between the BB and the Observation.
   */
  double dist_;

 public:
  /**
  * @brief Getter to know if the matching is valid or not.
  * 
  * @return true if the matching is valid.
  * @return false if the matching is not valid.
  */
  explicit operator bool() const { return valid_; }

  /**
   * @brief Match the implicit object to the BB with index \a bbInd and with
   * a matching distance \a dist.
   * 
   * @param bbInd Index of the BB
   * @param dist Matching distance
   */
  void match(size_t bbInd, double dist) {
    valid_ = true;
    bbInd_ = bbInd;
    dist_ = dist;
  }

  /**
     * @brief Invalidates the matching.
     */
  void unmatch() { valid_ = false; }

  /**
     * @brief Getter for \a bbInd_.
     * 
     * @return The index of the matched BB.
     * note: if not \a valid_, the result of this method is invalid as well.
     */
  const size_t &bbInd() const { return bbInd_; }

  /**
     * @brief Getter for \a dist_.
     * 
     * @return The matching distance between the implicit object and
     * the BB with index = \a bbInd_.
     * note: if not \a valid_, the result of this method is invalid as well.
     */
  const double &dist() const { return dist_; }
};

#endif  // MODULES_MATCHER_MATCHING_HPP