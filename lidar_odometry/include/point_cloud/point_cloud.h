///
/// @file point_cloud.h
/// @author Gergo Papp
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
    /// @brief Reads the input file
    /// 
    /// @param ifs Input file stream
    ///
    void read(std::ifstream& ifs);
    
    ///
    /// @brief Writes data to file
    /// 
    /// @param ofs Output file stream
    ///
    void write(std::ofstream& ofs);

    ///
    /// @brief Adds a single point to the PointCloud
    /// 
    /// @param point Point to be added to the PointCloud
    ///
    inline void add_point(const cv::Mat& point) { m_points.push_back(point); };

    ///
    /// @brief Calculates the mean of the PointCloud
    /// 
    /// @return cv::Vec3f Mean of the PointCloud
    ///
    cv::Vec3f get_mean() const;

    ///
    /// @brief Returns the number of points in the PointCloud
    /// 
    /// @return int Number of points
    ///
    inline int size() const { return m_points.rows; }

    ///
    /// @brief Returns if the cloud is empty or not
    /// 
    /// @return bool
    ///
    inline bool empty() const { return m_points.empty(); }

    ///
    /// @brief Gets the m_points object
    /// 
    /// @return const cv::Mat& Const reference to the m_points object
    ///
    const cv::Mat& get_points() const { return m_points; }

    ///
    /// @brief Transforms the PointCloud
    /// 
    /// @param T Transformation matrix
    ///
    void transform_cloud(const cv::Mat& T);

    ///
    /// @brief Copy assignment operator
    /// 
    /// @param other PointCloud which is going to be copied
    /// @return PointCloud& Reference to the PointCloud on which the copy assignment was called
    ///
    PointCloud& operator = (const PointCloud& other);

    friend std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
    friend std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);

private:
    cv::Mat m_points;     ///< Stores all of the points in a matrix format
};

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud);
std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud);
inline bool operator == (const cv::Point3f& lhs, const cv::Point3f& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

///
/// @brief Calculates the Euclidean Distance from origin
/// 
/// @param point Point in space
/// @return float Euclidean Distance from origin
///
inline float dist_from_origin(const cv::Vec3f& point) { return sqrt(pow(point[0], 2) + pow(point[1], 2) + pow(point[2], 2)); }

///
/// @brief Calculates the Euclidean Distance between two points
/// 
/// @param point1 Point1 in space
/// @param point2 Point2 in space
/// @return float Euclidean Distance between two points
///
inline float point_to_point_distance(const cv::Vec3f& point1, const cv::Vec3f& point2) { return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2)); }

///
/// @brief Estimates the implicit parameters of plane
/// 
/// @param points Vector of points which form a plane
/// @return std::vector<float> Implicit parameters of plane
///
std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points);

///
/// @brief Implementation of the basic ICP algorithm.
/// 
/// @param prev_cloud Previous PointCloud
/// @param cur_cloud Current PointCloud
/// @return cv::Mat Transformation between the two clouds
///
cv::Mat vanilla_icp(const PointCloud& prev_cloud, PointCloud cur_cloud);

///
/// @brief Implementation of the RANSAC algorithm.
/// 
/// @param points Input points of the algorithm
/// @param ransac_iter Number of iterations
/// @param threshold Threshold value for the algorithm
/// @return std::vector<cv::Point3f> Vector of points of the final inliers
///
std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter = 50, const float threshold = 0.2f);

///
/// @brief Implementation of smallest distance point to point association for ICP
/// TODO: Use KD-Trees for association
///
/// @param cloud1 First PointCloud
/// @param cloud2 Second PointCloud
/// @return cv::Mat Associated points from the two PointClouds
///
cv::Mat associate(const PointCloud& cloud1, const PointCloud& cloud2);

///
/// @brief Removes the ground plane
/// TODO: Implement a more sophisticated algorithm for ground plane removal
///
/// @param cloud PointCloud from which to remove the ground plane
///
void remove_ground_plane(std::unordered_set<cv::Point3f>& cloud);

// PointCloud clean_cloud(const PointCloud& cloud);

///
/// @brief Checks if file exists
/// 
/// @param path Path to file
/// @return bool
///
bool file_exists(const std::string path);

///
/// @brief Serializes PointCloud data to PLY file
/// 
/// @param path Path to output file
/// @param cloud PointCloud to serialize
/// @param color Color of points
///
void write_ply(const std::string path, const cv::Mat& cloud, const cv::Point3i color);
