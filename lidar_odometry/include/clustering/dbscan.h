#pragma once

#include "point_cloud/point_cloud.h"
#include <opencv2/opencv.hpp>

#include <vector>

void dbscan(const PointCloud& point_cloud, float epsilon, int min_points, std::vector<PointCloud>& segments);
void expand_cluster();

cv::Point3f get_core_point(const PointCloud& point_cloud);

