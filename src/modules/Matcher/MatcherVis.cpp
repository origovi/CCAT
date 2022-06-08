#include "modules/Matcher/MatcherVis.hpp"

/* --------------------------------- Private -------------------------------- */

std_msgs::ColorRGBA MatcherVis::BBColorToStd(const double &BBColor) {
  std_msgs::ColorRGBA res;
  res.a = 1.0;
  if (BBColor == 0) {
    res.r = 1.0;
    res.g = 1.0;
    res.b = 0.2;
  } else if (BBColor == 1) {
    res.r = 0.0;
    res.g = 0.0;
    res.b = 1.0;
  } else {
    res.r = 1.0;
    res.g = 0.56;
    res.b = 0.0;
  }
  return res;
}

cv::Scalar MatcherVis::BBColorToCV(const double &BBColor) {
  if (BBColor == 0)
    return CV_RGB(255, 255, 50);
  else if (BBColor == 1)
    return CV_RGB(0, 0, 255);
  else
    return CV_RGB(255, 145, 0);
}

/* --------------------------------- Public --------------------------------- */

MatcherVis::MatcherVis(const Params::Matcher &params) : params_(params) {}

void MatcherVis::publishPCLs(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &pcls) {
  pcl::PointCloud<pcl::PointXYZI> pclToPaint;
  for (const PCL::Ptr &pcl : pcls) {
    pclToPaint += *pcl;
  }
  sensor_msgs::PointCloud2 pcl;
  pcl::toROSMsg(pclToPaint, pcl);
  pcl.header.frame_id = "map";
  pcl.header.stamp = ros::Time::now();
  pubPcl(params_.topics.output.pcl).publish(pcl);
}

void MatcherVis::publishProjectedImg(const std::vector<std::pair<std::vector<cv::Point2d>, cv::Point2d>> &projections,
                                     const geometry_msgs::PoseArray &bbs,
                                     const std::vector<Matching> &matchings) {
  cv::Mat image(params_.image_height, params_.image_width, CV_8UC3, cv::Scalar(176, 176, 176));

  struct RGB {
    int R = rand() % 256;
    int G = rand() % 256;
    int B = rand() % 256;
    cv::Scalar scalarBGR() { return CV_RGB(R, G, B); }
  };
  
  std::vector<RGB> colors(bbs.poses.size());

  std::vector<bool> paintedBBs(bbs.poses.size(), false);

  // Paint the matched BB(s) and
  for (size_t i = 0; i < projections.size(); i++) {
    if (bool(matchings[i])) {
      paintedBBs[matchings[i].bbInd()] = true;
      for (const cv::Point2d &p : projections[i].first) {
        if (params_.colors_in_projected)
          cv::circle(image, p, 2, colors[matchings[i].bbInd()].scalarBGR(), -1);
        else
          cv::circle(image, p, 2, BBColorToCV(bbs.poses[matchings[i].bbInd()].position.x), -1);
      }
      cv::Point2d pt1(bbs.poses[matchings[i].bbInd()].orientation.x, bbs.poses[matchings[i].bbInd()].orientation.y);
      cv::Point2d pt2(bbs.poses[matchings[i].bbInd()].orientation.z, bbs.poses[matchings[i].bbInd()].orientation.w);
      if (params_.colors_in_projected)
        cv::rectangle(image, pt1, pt2, colors[matchings[i].bbInd()].scalarBGR(), 2, 8, 0);
      else
        cv::rectangle(image, pt1, pt2, BBColorToCV(bbs.poses[matchings[i].bbInd()].position.x), 2, 8, 0);

    } else {
      for (const cv::Point2d &p : projections[i].first) {
        cv::circle(image, p, 2, CV_RGB(0, 0, 0), -1);
      }
    }
    // Paint the centroid of the observation (in Green)
    cv::circle(image, projections[i].second, 4, CV_RGB(0, 255, 0), -1);
  }

  // Paint the not matched BB(s) in white
  for (size_t i = 0; i < bbs.poses.size(); i++) {
    if (!paintedBBs[i]) {
      cv::Point2d pt1(bbs.poses[i].orientation.x, bbs.poses[i].orientation.y);
      cv::Point2d pt2(bbs.poses[i].orientation.z, bbs.poses[i].orientation.w);
      cv::rectangle(image, pt1, pt2, CV_RGB(255, 255, 255), 2, 8, 0);
    }
  }

  // The timestamp will be taken from the bbs message header
  const sensor_msgs::ImagePtr &msg = cv_bridge::CvImage(bbs.header, "bgr8", image).toImageMsg();

  pubImg(params_.topics.output.projected).publish(msg);
}

void MatcherVis::publishMatchingMarkers(const std::vector<geometry_msgs::Point> &projsCentroids,
                                        const geometry_msgs::PoseArray &bbs,
                                        const std::vector<Matching> &matchings) {
  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(1 + matchings.size());
  visualization_msgs::Marker m;
  m.pose.orientation.w = 1.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.type = visualization_msgs::Marker::CYLINDER;
  m.header.frame_id = "base_link";
  m.header.stamp = ros::Time::now();
  size_t id = 0;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = id++;
  ma.markers.push_back(m);
  m.action = visualization_msgs::Marker::ADD;

  for (size_t i = 0; i < matchings.size(); i++) {
    if (bool(matchings[i])) {
      m.id = id++;
      m.pose.position = projsCentroids[i];
      m.color = BBColorToStd(bbs.poses[matchings[i].bbInd()].position.x);
      ma.markers.push_back(m);
    }
  }

  pubMA(params_.topics.output.instant_markers).publish(ma);
}