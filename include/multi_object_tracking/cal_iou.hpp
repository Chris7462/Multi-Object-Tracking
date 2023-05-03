#pragma once

#include "multi_object_tracking/tracker.hpp"


//https://blog.csdn.net/zjn295771349/article/details/112989543
static int find_leftmost_point(std::vector<cv::Point2f>& intersectingRegion)
{
  int index = 0;
  float tmp = intersectingRegion[0].x;
  for(int i=1; i<intersectingRegion.size(); i++) {
    if(intersectingRegion[i].x < tmp) {
      tmp = intersectingRegion[i].x;
      index = i;
    }
  }
  return index;
}

static std::vector<cv::Point2f> sort_points(std::vector<cv::Point2f>& intersectingRegion)
{
  std::vector<cv::Point2f> sort_intersectingRegion;
  int leftmost_index = find_leftmost_point(intersectingRegion);

  std::vector<float> arctan;
  for(int i=0; i<intersectingRegion.size(); i++) {
    arctan.push_back(atan2(intersectingRegion[i].x - intersectingRegion[leftmost_index].x, intersectingRegion[i].y - intersectingRegion[leftmost_index].y));
  }

  std::vector<int> index;
  for(int i=0; i<arctan.size(); i++) {
    index.push_back(i);
  }

  std::sort(index.begin(), index.end(), [&](const int& a, const int& b) {return (arctan[a] < arctan[b]);});

  for(int i=0; i<index.size(); i++) {
    sort_intersectingRegion.push_back(intersectingRegion[index[i]]);
  }
  return sort_intersectingRegion;
}

inline float RectIou(const cv::RotatedRect& r1, const cv::RotatedRect& r2)
{
  std::vector<cv::Point2f> intersectingRegion;
  float intersectionType = cv::rotatedRectangleIntersection(r1, r2, intersectingRegion);
  float inter_area;

  if (intersectingRegion.empty()){
    inter_area = 0;
  } else {
    std::vector<cv::Point2f> sort_intersectingRegion = sort_points(intersectingRegion);
    inter_area = cv::contourArea(sort_intersectingRegion);
  }

  cv::Size2f r1size = r1.size;
  cv::Size2f r2size = r2.size;
  float area1 = r1size.area();
  float area2 = r2size.area();
  float iou = inter_area/(area1 + area2 - inter_area+ 0.00000001);
  return 1-iou;
};
