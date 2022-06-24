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

class PointCloud {
public:
    PointCloud() = default;
    PointCloud(const PointCloud& other);
    PointCloud(std::ifstream& ifs);
    ~PointCloud() = default;

    void read(std::ifstream& ifs);
    void write(std::ofstream& ofs);

    inline void add_point(const cv::Mat& point) { points.push_back(point); };
    // void remove_point(const cv::Point3f& point);  
    // void remove_point(std::unordered_set<cv::Point3f>::const_iterator it); 

    cv::Vec3f center_of_mass() const;
    inline int size() const      { return points.rows; }
    inline bool empty() const       { return points.empty(); }
    const cv::Mat& get_points() const { return points; }

    PointCloud& operator = (const PointCloud& other);
    friend std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
    friend std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

private:
    cv::Mat points;
};

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);
inline bool operator == (const cv::Point3f& lhs, const cv::Point3f& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

inline float dist_from_origin(const cv::Vec3f& point) { return sqrt(pow(point[0], 2) + pow(point[1], 2) + pow(point[2], 2)); }
// inline float point_to_point_distance(const cv::Vec3f& point1, const cv::Vec3f& point2) { return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2)); }

std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points);
cv::Mat vanilla_icp(const PointCloud& prev_cloud, const PointCloud& cur_cloud);
std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter = 50, const float threshold = 0.2f);

// std::vector<PointAssociation> associate(const PointCloud& cloud1, const PointCloud& cloud2);
PointCloud remove_ground_plane(const PointCloud& cloud);
void remove_ground_plane(PointCloud& cloud);
// PointCloud clean_cloud(const PointCloud& cloud);

void write_ply(std::ofstream& ofs, const PointCloud& cloud, const cv::Point3i color);
