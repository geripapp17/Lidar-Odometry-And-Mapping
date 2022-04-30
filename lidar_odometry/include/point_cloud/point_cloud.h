#pragma once

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <fstream>
#include <math.h>
#include <unordered_set>

#define LOWEST_DISTANCE 0.3

namespace std {

    template <>
    struct hash<cv::Point3f> {

        std::size_t operator()(const cv::Point3f& point) const {
        using std::size_t;
        using std::hash;

        return ((hash<float>()(point.x)
                ^ (hash<float>()(point.y) << 1)) >> 1)
                ^ (hash<float>()(point.z) << 1);
        }
    };
}

class PointCloud {
public:
    PointCloud() = default;
    PointCloud(const PointCloud& other);
    PointCloud(std::ifstream& ifs);
    ~PointCloud() = default;

    void read(std::ifstream& ifs);
    void write(std::ofstream& ofs);

    inline void add_point(const cv::Point3f& point) { points.insert(point); }
    inline void remove_point(const cv::Point3f& point) { points.erase(point); }
    inline void remove_point(std::unordered_set<cv::Point3f>::const_iterator it) { points.erase(it); }

    inline size_t size() const { return points.size(); }
    inline const std::unordered_set<cv::Point3f>& get_points() const { return points; } 

    friend std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
    friend std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

    std::unordered_set<cv::Point3f>::iterator begin() { return points.begin(); }
    std::unordered_set<cv::Point3f>::iterator end() { return points.end(); }
    std::unordered_set<cv::Point3f>::const_iterator begin() const { return points.cbegin(); }
    std::unordered_set<cv::Point3f>::const_iterator end() const { return points.cend(); }

private:
    std::unordered_set<cv::Point3f> points;
};

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);
inline bool operator == (const cv::Point3f& lhs, const cv::Point3f& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

inline float dist_from_origin(const cv::Point3f& point) { return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)); }
inline float point_to_point_distance(const cv::Point3f& point1, const cv::Point3f& point2) { return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2)); }

std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points);
std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter = 50, const float threshold = 0.2f);
void remove_ground_plane(PointCloud& cloud);

void write_ply(std::ofstream& ofs, const PointCloud& cloud);
void write_ply(std::ofstream& ofs, const std::vector<PointCloud>& clusters);