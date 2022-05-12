#include "point_cloud/point_cloud.h"
#include "clustering/dbscan.h"

#include <time.h>
#include <limits>

PointCloud::PointCloud(const PointCloud& other) {

    points = other.points;
}

PointCloud::PointCloud(std::ifstream& ifs) {

    read(ifs);
}

void PointCloud::read(std::ifstream& ifs) { ifs >> *this; }

void PointCloud::write(std::ofstream& ofs) { ofs << *this; }

PointCloud& PointCloud::operator = (const PointCloud& other) {
    points = other.points;
    return *this;
}

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

void write_ply(std::ofstream& ofs, const PointCloud& cloud, const cv::Point3i color) {

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

    for(const auto& point : cloud) {
        ofs << point.x << " " << point.y << " " << point.z << " " << color.x << " " << color.y << " " << color.z << "\n";
    }
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

        // for(const auto& point : cluster) {
        //     ofs << point.x << " " << point.y << " " << point.z << " " << r << " " << g << " " << b << "\n";
        // }
        ofs << cluster.center_of_mass().x << " " << cluster.center_of_mass().y << " " << cluster.center_of_mass().z << " " << r << " " << g << " " << b << "\n";

    }
}

cv::Point3f PointCloud::center_of_mass() const {

    cv::Point3f p0 = { 0.0, 0.0, 0.0 };
    for (const auto& point : points) {
        p0 += point;
    }

    p0.x /= points.size();
    p0.y /= points.size();
    p0.z /= points.size();

    return p0;
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

PointCloud clean_cloud(const PointCloud& cloud) {

    // Remove ground plane
    PointCloud cloud_ground_removed = remove_ground_plane(cloud);

    std::vector<PointCloud> clusters;
    dbscan(cloud_ground_removed, 1.0, 4, clusters);

    // Remove clusters which contain less points than a given threshold
    clusters.erase(std::remove_if(clusters.begin(),
                                clusters.end(),
                                [](const PointCloud& cloud) { return cloud.size() <= 10; }),
                   clusters.end());

    PointCloud result;
    for (const auto& cluster : clusters) {
        result.add_point(cluster.center_of_mass());
    }

    return result;
}

std::vector<PointAssociation> associate(const PointCloud& source_cloud, const PointCloud& dest_cloud) {

    std::vector<PointAssociation> associations;

    for (const auto& source_point : source_cloud) {
        float min_dist = std::numeric_limits<float>::max();
        cv::Point3f best_match;
        
        for (const auto& dest_point : dest_cloud) {
            float dist = dist_from_origin(source_point - dest_point);
            if(dist < min_dist) {
                min_dist = dist;
                best_match = dest_point;
            }
        }

        associations.push_back(PointAssociation {source_point, best_match});
    }

    return associations;
}

Transformation vanilla_icp(const PointCloud& source_cloud, const PointCloud& dest_cloud) {

    PointCloud source_cleaned = clean_cloud(source_cloud);
    PointCloud dest_cleaned = clean_cloud(dest_cloud);

    Transformation transf { cv::Mat::eye(3, 3, CV_32F), cv::Mat::zeros(3, 1, CV_32F) };

    std::ofstream ofs{ "/home/geri/work/c++/Lidar-Odometry/output/dest_cleaned.ply", std::ofstream::out };
    write_ply(ofs, dest_cleaned, { 255, 0, 0 });
    ofs.close();

    for (int i = 0; i < 10; ++i) {

        std::ofstream ofs2{ "/home/geri/work/c++/Lidar-Odometry/output/source_cleaned" + std::to_string(i) + ".ply", std::ofstream::out };
        write_ply(ofs2, source_cleaned, { 0 , 255, 0 });
        ofs2.close();

        cv::Point3f source_mean = source_cleaned.center_of_mass();
        cv::Point3f dest_mean = dest_cleaned.center_of_mass();

        std::vector<PointAssociation> associations = associate(source_cleaned, dest_cleaned);

        cv::Mat H = cv::Mat::zeros(3,3, CV_32F);
        for (const auto& association : associations) {
            cv::Point3f source_point_shifted = association.source_point - source_mean;
            cv::Point3f dest_point_shifted = association.dest_point - dest_mean;
            std::vector<float> source_point_vec { source_point_shifted.x, source_point_shifted.y, source_point_shifted.z };
            std::vector<float> dest_point_vec { dest_point_shifted.x, dest_point_shifted.y, dest_point_shifted.z };

            cv::Mat source_point_mat { source_point_vec };
            cv::Mat dest_point_mat { dest_point_vec };

            H += (source_point_mat * dest_point_mat.t());
        }

        cv::SVD decomp(H, cv::SVD::FULL_UV);
        cv::Mat R = decomp.vt.t() * decomp.u.t() ;

        std::cout << R << std::endl << cv::Mat { dest_mean } << std::endl;

        cv::Mat T = cv::Mat { dest_mean } - R * cv::Mat { source_mean };

        std::cout << T << std::endl;

        PointCloud tmp;
        for(const auto& point : source_cleaned) {
            cv::Mat point_mat { { point.x, point.y, point.z } };
            cv::Mat point_transf = R * point_mat + T;
            tmp.add_point(cv::Point3f { point_transf.at<float>(0), point_transf.at<float>(1), point_transf.at<float>(2) });
        }

        source_cleaned = tmp;
        transf.R = R * transf.R;
        transf.T = R * transf.T + T;
    }

    return transf;
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

PointCloud remove_ground_plane(const PointCloud& cloud) {
    // TODO: Implement a more sophisticated algorithm for ground plane removal

    PointCloud result = cloud;

    srand(time(0));

    // Select points from the point cloud which z coordinate is less than -1
    std::vector<cv::Point3f> points;
    for(const auto& point : result) {
        if(point.z < -1.0) {
            points.push_back(point);
        }
    }

    std::vector<cv::Point3f> ground_plane = ransac(points);
    
    for(const auto& point : ground_plane) {
        result.remove_point(point);
    }

    return result;
}