#ifndef MY_DEPTH_ILLUSTRATOR
#define MY_DEPTH_ILLUSTRATOR

#include <opencv/cv.h>

cv::Mat myGetRgbDepthMap(const cv::Mat& depth_map, int min_dst, int rg, int gb, int max_dst);
cv::Mat myGetDispMap(const cv::Mat& disp_map);

#endif
