#include "point_cloud/point_cloud.h"

#include <time.h>

PointCloud::PointCloud(const PointCloud& other) {

    points = other.points;
}

PointCloud::PointCloud(std::ifstream& ifs) {

    read(ifs);
}

void PointCloud::read(std::ifstream& ifs) { ifs >> *this; }

void PointCloud::write(std::ofstream& ofs) { ofs << *this; }

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud) {

    for(const auto& point : cloud.points) {
        
    }

    return ofs;
}

std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud) {

    std::string line;
    while(std::getline(ifs, line)) {

        std::istringstream iss{ line };
        float x, y, z;

        iss >> x >> y >> z;
        cv::Point3f p{ x, y, z };
        
        if(dist_from_origin(p) > LOWEST_DISTANCE) {
            cloud.points.insert(p);
        }
    }

    return ifs;
}

void write_ply(std::ofstream& ofs, const PointCloud& cloud) {

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex "<< cloud.size() << std::endl;
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";
}

void write_ply(std::ofstream& ofs, const std::vector<PointCloud>& clusters) {

    int sum_points = 0;
    for(const auto& cluster : clusters) {
        sum_points += cluster.size();
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex "<< sum_points << std::endl;
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    for(const auto& cluster : clusters) {

        int r = (rand() + 100) % 256;
        int g = (rand() + 100) % 256;
        int b = (rand() + 100) % 256;

        for(const auto& point : cluster) {
            ofs << point.x << " " << point.y << " " << point.z << " " << r << " " << g << " " << b << "\n";
        }
    }
}

std::vector<float> estimate_plane_implicit(const std::vector<cv::Point3f>& points) {

    cv::Point3f p0 { 0.0, 0.0, 0.0 };
    for(const auto& point : points) {
        p0 += point;
    }

    p0.x /= points.size();
    p0.y /= points.size();
    p0.z /= points.size();

    cv::Mat A { points.size(), 3, CV_32F };
    for (int i = 0; i < points.size(); i++) {
        A.at<float>(i, 0) = points.at(i).x - p0.x;
        A.at<float>(i, 1) = points.at(i).y - p0.y;
        A.at<float>(i, 2) = points.at(i).z - p0.z;
    }

    cv::Mat M = A.t() * A;
    cv::Mat eigen_vals, eigen_vecs;
    cv::eigen(M, eigen_vals, eigen_vecs);

    float a = eigen_vecs.at<float>(2, 0);
    float b = eigen_vecs.at<float>(2, 1);
    float c = eigen_vecs.at<float>(2, 2);
    float d = -1 * (a * p0.x + b * p0.y + c * p0.z);
    
    std::vector<float> ret {a, b, c, d};

    return ret;
}

std::vector<cv::Point3f> ransac(const std::vector<cv::Point3f>& points, const int ransac_iter, const float threshold) {

    int best_inlier_num = 0;
    std::vector<float> best_plane { 4, 0.0 };
    std::vector<cv::Point3f> final_inliers;

    for (int i = 0; i < ransac_iter; ++i) {
        // Choose 3 random coordinates for RANSAC
        cv::Point3f p1 = points[rand() % points.size()];
        cv::Point3f p2 = points[rand() % points.size()];
        while(p1 == p2) { p2 = points[rand() % points.size()]; }
        cv::Point3f p3 = points[rand() % points.size()];
        while(p2 == p3) { p3 = points[rand() % points.size()]; }
        
        std::vector<cv::Point3f> pts {p1, p2, p3};
        std::vector<float> plane_params = estimate_plane_implicit(pts);

        int num_inliers = 0;
        std::vector<cv::Point3f> inliers;
        for(const auto& point : points) {
            float dist = fabs(plane_params[0] * point.x + plane_params[1] * point.y + plane_params[2] * point.z + plane_params[3]);

            if(dist < threshold) {
                ++num_inliers;
                inliers.push_back(point);
            }
        }

        if (num_inliers > best_inlier_num) {
            best_inlier_num = num_inliers;
            final_inliers = inliers;
        }
    }
    
    return final_inliers;
}

void remove_ground_plane(PointCloud& cloud) {
    // TODO: Implement a more sophisticated algorithm for ground plane removal

    srand(time(0));

    // Select points from the point cloud which z coordinate is less than -1
    std::vector<cv::Point3f> points;
    for(const auto& point : cloud) {
        if(point.z < -1.0) {
            points.push_back(point);
        }
    }

    std::vector<cv::Point3f> ground_plane = ransac(points);
    
    for(const auto& point : ground_plane) {
        cloud.remove_point(point);
    }
}
