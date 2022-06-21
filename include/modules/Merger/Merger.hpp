/**
 * @file Merger.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Merger module, in charge of
 * merging the decisions of all the cameras and keeping the most probable one.
 * E.g. If two Matcher see (and match) the same cone with a different type of
 * bounding box, which type is the truth?
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_MERGER_HPP
#define MODULES_MERGER_HPP

#include <ros/ros.h>

#include <map>
#include <vector>

#include "structures/ConeUpdate.hpp"
#include "structures/Params.hpp"

/**
 * @brief Merges all Matchers' output into a single vector of ConeUpdate.
 * 
 */
class Merger {
 private:
  /* -------------------------- Private Constructor ------------------------- */

  Merger();

  /* -------------------------- Private Attributes -------------------------- */
  
  /**
   * @brief The parameters object of the Merger, it includes all necessary
   * paramters that will be used in the module.
   */
  Params::Merger params_;

  /**
   * @brief This iteration's merged cones. Can be get with Merger::getData().
   */
  std::vector<ConeUpdate> currentCones_;

 public:
  /* --------------------------- Singleton Pattern -------------------------- */

  static Merger &getInstance();
  Merger(Merger const &) = delete;
  void operator=(Merger const &) = delete;

  /**
   * @brief It initializes the Merger.
   * 
   * @param[in] params The params that will be used by the Merger
   */
  void init(const Params::Merger &params);

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Main function of the Merger class, it executes all the functions in
   * order to relate all Matcher(s) output and decide which ConeUpdate is the
   * most probable.
   * 
   * It look for cones with the same \a id and chooses which one has more
   * probabilities of being correct according to the matching distance
   * 
   * @param[in] conesLeft Cones outputted by the left Merger
   * @param[in] conesRight Cones outputted by the right Matcher
   */
  void run(const std::vector<ConeUpdate> &conesLeft, const std::vector<ConeUpdate> &conesRight);

  /**
   * @brief Getter for this iteration's merged result.
   * 
   * @return currentCones_
   */
  const std::vector<ConeUpdate> &getData() const;
};

#endif  // MODULES_MERGER_HPP