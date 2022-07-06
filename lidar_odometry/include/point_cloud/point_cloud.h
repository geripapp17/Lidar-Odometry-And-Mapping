///
/// @file point_cloud.h
/// @author Gergo Papp (geri.papp17@gmail.com)
/// @brief 
/// @date 2022-07-06
/// 
/// @copyright Copyright (c) 2022
///

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

///
/// @brief 
/// 
///
class PointCloud {
public:
    ///
    /// @brief Constructs a new PointCloud object
    /// 
    PointCloud() = default;
    
    ///
    /// @brief Constructs a new PointCloud object
    /// 
    /// @param other 
    ///
    PointCloud(const PointCloud& other);

    ///
    /// @brief Constructs a new PointCloud object
    /// 
    /// @param ifs 
    ///
    PointCloud(std::ifstream& ifs);

    ///
    /// @brief Destroys the PointCloud object
    ///
    ~PointCloud() = default;

    ///
    /// @brief 
    /// 
    /// @param ifs 
    ///
    void read(std::ifstream& ifs);
    
    ///
    /// @brief 
    /// 
    /// @param ofs 
    ///
    void write(std::ofstream& ofs);

    ///
    /// @brief 
    /// 
    /// @param point 
    ///
    inline void add_point(const cv::Mat& point) { points.push_back(point); };

    // void remove_point(const cv::Point3f& point);  
    // void remove_point(std::unordered_set<cv::Point3f>::const_iterator it); 

    ///
    /// @brief 
    /// 
    /// @return cv::Vec3f 
    ///
    cv::Vec3f center_of_mass() const;

    ///
    /// @brief 
    /// 
    /// @return int 
    ///
    inline int size() const { return points.rows; }

    ///
    /// @brief 
    /// 
    /// @return true 
    /// @return false 
    ///
    inline bool empty() const { return points.empty(); }

    ///
    /// @brief Get the _points object
    /// 
    /// @return const cv::Mat& 
    ///
    const cv::Mat& get_points() const { return points; }

    ///
    /// @brief 
    /// 
    /// @param T 
    ///
    void transform_cloud(const cv::Mat& T);

    ///
    /// @brief 
    /// 
    /// @param other 
    /// @return PointCloud& 
    ///
    PointCloud& operator = (const PointCloud& other);

    friend std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
    friend std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

private:
    cv::Mat points;     ///< 
};

///
/// @brief 
/// 
/// @param ofs 
/// @param cloud 
/// @return std::ofstream& 
///
std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);

///
/// @brief 
/// 
/// @param ifs 
/// @param cloud 
/// @return std::ifstream& 
///
std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

///
/// @brief 
/// 
/// @param lhs 
/// @param rhs 
/// @return 
///
inline bool operator == (const cv::Point3f& lhs, const cv::Point3f& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

///
/// @brief 
/// 
/// @param point 
/// @return float 
///
inline float dist_from_origin(const cv::Vec3f& point) { return sqrt(pow(point[0], 2) + pow(point[1], 2) + pow(point[2], 2)); }

///
/// @brief 
/// 
/// @param point1 
/// @param point2 
/// @return float 
///
inline float point_to_point_distance(const cv::Vec3f& point1, const cv::Vec3f& point2) { return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2)); }

///
/// @brief 
/// 
/// @param points 
/// @return std::vector<float> 
///
std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points);

///
/// @brief 
/// 
/// @param prev_cloud 
/// @param cur_cloud 
/// @return cv::Mat 
///
cv::Mat vanilla_icp(const PointCloud& prev_cloud, PointCloud cur_cloud);

///
/// @brief 
/// 
/// @param points 
/// @param ransac_iter 
/// @param threshold 
/// @return std::vector<cv::Point3f> 
///
std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter = 50, const float threshold = 0.2f);

///
/// @brief 
/// TODO: Use KD-Trees for association
///
/// @param cloud1 
/// @param cloud2 
/// @return cv::Mat 
///
cv::Mat associate(const PointCloud& cloud1, const PointCloud& cloud2);

///
/// @brief 
/// TODO: Implement a more sophisticated algorithm for ground plane removal
///
/// @param cloud 
///
void remove_ground_plane(std::unordered_set<cv::Point3f>& cloud);

// PointCloud clean_cloud(const PointCloud& cloud);

///
/// @brief 
/// 
/// @param path 
/// @return 
///
bool file_exists(const std::string path);

///
/// @brief 
/// 
/// @param path 
/// @param cloud 
/// @param color 
///
void write_ply(const std::string path, const cv::Mat& cloud, const cv::Point3i color);
