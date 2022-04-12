#include "clustering/dbscan.h"

#include <time.h>
#include <queue>
#include <unordered_set>
#include <optional>


void dbscan(const PointCloud& point_cloud, const float epsilon, const int min_points, std::vector<PointCloud>& clusters) {

    srand(time(NULL));
    
    std::unordered_set<cv::Point3f> unvisited{ point_cloud.get_points() };
    std::unordered_set<cv::Point3f> outliers;

    while (!unvisited.empty()) {

        // select random element of the set
        auto it = unvisited.cbegin();
        std::advance(it, rand() % unvisited.size());

        auto cluster = expand_cluster(point_cloud, unvisited, outliers, *it, epsilon, min_points);

        if(cluster.has_value()) {
            unvisited.erase(*it);
            for(const auto& p : *cluster) {
                outliers.erase(p);
            }

            clusters.push_back(*cluster);
        }
        else {
            outliers.insert(*it);
            unvisited.erase(*it);
        }
    }
}

std::optional<PointCloud> expand_cluster(const PointCloud& point_cloud, std::unordered_set<cv::Point3f>& unvisited, const std::unordered_set<cv::Point3f>& outliers, const cv::Point3f& first_point, const float epsilon, const int min_points) {

    auto point_queue = get_neighbours(point_cloud, first_point, epsilon);

    if (point_queue.size() >= min_points) {
        PointCloud cluster;
        cluster.add_point(first_point);

        while (!point_queue.empty()) {
            auto curr_point = point_queue.front();
            point_queue.pop();

            cluster.add_point(curr_point);
            auto curr_point_neighbours = get_neighbours(point_cloud, curr_point, epsilon);

            if(curr_point_neighbours.size() >= min_points) {
                while(!curr_point_neighbours.empty()) {
                    auto curr_neighbour = curr_point_neighbours.front();
                    curr_point_neighbours.pop();

                    auto it_unvisited = unvisited.find(curr_neighbour);
                    if (it_unvisited != unvisited.end()) {
                        point_queue.push(curr_neighbour);
                        unvisited.erase(curr_neighbour);
                        cluster.add_point(curr_neighbour);
                    }

                    auto it_outliers = outliers.find(curr_neighbour);
                    if(it_outliers != outliers.end()) {
                        cluster.add_point(curr_neighbour);
                    }
                }
            }
        }

        return cluster;
    }
    else { return std::nullopt; }
    
}

std::queue<cv::Point3f> get_neighbours(const PointCloud& point_cloud, const cv::Point3f& point, const float epsilon) {

    std::queue<cv::Point3f> neighbours;

    for(auto& p : point_cloud.get_points()) {
        if(point_to_point_distance(point, p) <= epsilon && point != p) {
            neighbours.push(p);
        }
    }

    return neighbours;
}

// https://github.com/james-yoo/DBSCAN/blob/master/dbscan.cpp