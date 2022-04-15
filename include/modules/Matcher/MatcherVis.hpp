#ifndef MATHCERVIS_HPP
#define MATHCERVIS_HPP

#include "utilities/Visualization.hpp"
#include "modules/Matcher/Matcher.hpp"

/**
 * @brief Visualization class for the Matcher module.
 * 
 */
class MatcherVis : private Visualization {
  const Params::Matcher params_;

  MatcherVis(ros::NodeHandle *const nh, const Params::Matcher &params);

  void publishProjectedImg();
};

#endif