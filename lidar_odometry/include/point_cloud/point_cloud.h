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

struct PointAssociation {
    cv::Point3f source_point;
    cv::Point3f dest_point;
};

struct Transformation {
    cv::Mat R;
    cv::Mat T;
};

class PointCloud {
public:
    using iterator = std::unordered_set<cv::Point3f>::iterator;
    using const_iterator = std::unordered_set<cv::Point3f>::const_iterator;

    PointCloud() = default;
    PointCloud(const PointCloud& other);
    PointCloud(std::ifstream& ifs);
    ~PointCloud() = default;

    void read(std::ifstream& ifs);
    void write(std::ofstream& ofs);

    inline void add_point(const cv::Point3f& point)                                 { points.insert(point); }
    inline void remove_point(const cv::Point3f& point)                              { points.erase(point); }
    inline void remove_point(std::unordered_set<cv::Point3f>::const_iterator it)    { points.erase(it); }

    cv::Point3f center_of_mass() const;
    inline size_t size() const      { return points.size(); }
    inline bool empty() const       { return !size(); }
    inline const std::unordered_set<cv::Point3f>& get_points() const { return points; } 

    PointCloud& operator = (const PointCloud& other);
    friend std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
    friend std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

    inline iterator begin()          { return points.begin(); }
    inline iterator end()            { return points.end(); }
    inline const_iterator begin() const    { return points.begin(); }
    inline const_iterator end() const      { return points.end(); }

    std::unordered_set<cv::Point3f>& get_points() { return points; }

private:
    std::unordered_set<cv::Point3f> points;
};

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);
inline bool operator == (const cv::Point3f& lhs, const cv::Point3f& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

inline float dist_from_origin(const cv::Point3f& point) { return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)); }
inline float point_to_point_distance(const cv::Point3f& point1, const cv::Point3f& point2) { return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2)); }

std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points);
Transformation vanilla_icp(const PointCloud& prev_cloud, const PointCloud& cur_cloud);
std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter = 50, const float threshold = 0.2f);

std::vector<PointAssociation> associate(const PointCloud& cloud1, const PointCloud& cloud2);
PointCloud remove_ground_plane(const PointCloud& cloud);
void remove_ground_plane(PointCloud& cloud);
PointCloud clean_cloud(const PointCloud& cloud);

void write_ply(std::ofstream& ofs, const PointCloud& cloud, const cv::Point3i color);
void write_ply(std::ofstream& ofs, const std::vector<PointCloud>& clusters);