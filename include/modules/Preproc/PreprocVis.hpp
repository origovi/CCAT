/**
 * @file PreprocVis.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It provides all the necessary tools to the Preproc module to
 * visualize all the data and debug.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef MODULES_PREPROC_PREPROCVIS_HPP
#define MODULES_PREPROC_PREPROCVIS_HPP

#include "structures/Params.hpp"
#include "structures/Observation.hpp"
#include "utils/Visualization.hpp"

/**
 * @brief Visualization class for the Preproc module.
 */
class PreprocVis : public Visualization {
 private:
  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief The parameters object of the Preproc, it includes all necessary
   * paramters that will be used in the module.
   */
  Params::Preproc params_;

 public:
  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief It initializes the submodule.
   * 
   * @param[in] params The params that will be used by the Preproc
   * (and PreprocVis)
   */
  void init(const Params::Preproc &params);

  /**
   * @brief Creates and publishes the markers corresponding to the 3D map
   * observed cones.
   * 
   * @param[in] observations
   */
  void publishInputMarkers(const std::vector<Observation> &observations);
};

#endif  // MODULES_PREPROC_PREPROCVIS_HPP