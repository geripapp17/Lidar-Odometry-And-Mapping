#pragma once

#include "point_cloud/point_cloud.h"
#include <opencv2/opencv.hpp>

#include <vector>

// void dbscan(const PointCloud& point_cloud, const float epsilon, const int min_points, std::vector<PointCloud>& clusters);
// std::optional<PointCloud> expand_cluster(const PointCloud& point_cloud, std::unordered_set<cv::Point3f>& unvisited, const std::unordered_set<cv::Point3f>& outliers, const cv::Point3f& first_point, const float epsilon, const int min_points);
// std::queue<cv::Point3f> get_neighbours(const PointCloud& point_cloud, const cv::Point3f& point, const float epsilon);
