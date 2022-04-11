#include "clustering/dbscan.h"

#include <time.h>
#include <queue>
#include <unordered_set>


void dbscan(const PointCloud& point_cloud, const float epsilon, const int min_points, std::vector<PointCloud>& clusters) {

    srand(time(NULL));
    
    std::unordered_set<cv::Point3f> unvisited{ point_cloud.get_points() };

    while (!unvisited.empty()) {

        // select random element of the set
        auto it = unvisited.cbegin();
        std::advance(it, rand() % unvisited.size());

        auto result = expand_cluster(point_cloud, unvisited, *it, epsilon, min_points);
    }


}

std::pair<PointCloud, bool> expand_cluster(const PointCloud& point_cloud, std::unordered_set<cv::Point3f>& unvisited, const cv::Point3f& first_point, const float epsilon, const int min_points) {

    PointCloud cluster;
    auto core_neighbours = get_neighbours(point_cloud, first_point, epsilon);

    if (core_neighbours.size() < min_points) {
        
    }
    else {
        for(const auto& p : core_neighbours) {
            cluster.add_point(p);

            auto point_neighbours = get_neighbours(point_cloud, p, epsilon);
            if(point_neighbours.size() >= min_points) {
                for(const auto& n : point_neighbours) {
                    if()
                }
            }
        }
    }
    
}

std::vector<cv::Point3f> get_neighbours(const PointCloud& point_cloud, const cv::Point3f& point, const float epsilon) {

    std::vector<cv::Point3f> neighbours;

    for(auto& p : point_cloud.get_points()) {
        if(point_to_point_distance(point, p) <= epsilon && point != p) {
            neighbours.push_back(p);
        }
    }

    return neighbours;
}

// https://github.com/james-yoo/DBSCAN/blob/master/dbscan.cpp