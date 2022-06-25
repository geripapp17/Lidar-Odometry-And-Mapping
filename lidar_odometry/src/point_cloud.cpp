#include "point_cloud/point_cloud.h"
#include "clustering/dbscan.h"

#include <time.h>
#include <limits>

PointCloud::PointCloud(const PointCloud& other) {

    points = other.points.clone();
}

PointCloud::PointCloud(std::ifstream& ifs) {

    read(ifs);
}

void PointCloud::read(std::ifstream& ifs) { ifs >> *this; }

void PointCloud::write(std::ofstream& ofs) { ofs << *this; }

PointCloud& PointCloud::operator = (const PointCloud& other) {
    
    points = other.points.clone();
    return *this;
}

std::ofstream& operator << (std::ofstream& ofs, const PointCloud& cloud) {

    // for(const auto& point : cloud.points) {
        
    // }

    return ofs;
}

std::ifstream& operator >> (std::ifstream& ifs, PointCloud& cloud) {

    std::unordered_set<cv::Point3f> filteredPoints;

    std::string line;
    while(std::getline(ifs, line)) {

        std::istringstream iss{ line };
        cv::Vec3f p;

        iss >> p[0] >> p[1] >> p[2];

        if(dist_from_origin(p) > LOWEST_DISTANCE) {
            if(p[2] < -1.0) { filteredPoints.insert(p); }
            else { cloud.add_point( cv::Mat{ p }.t() ); }
        }
    }

    remove_ground_plane(filteredPoints);
    for(const auto& point : filteredPoints) {
        cv::Vec3f p = { point.x, point.y, point.z };
        cloud.add_point( cv::Mat{ p }.t() );
    }

    return ifs;
}

void remove_ground_plane(std::unordered_set<cv::Point3f>& points) {
    // TODO: Implement a more sophisticated algorithm for ground plane removal

    srand(time(0));

    std::vector<cv::Point3f> point_vec;
    for(const auto& point : points) {
        point_vec.push_back(point);
    }

    std::vector<cv::Point3f> ground_plane = ransac(point_vec);

    for(const auto& point : ground_plane) {
        points.erase(point);
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

cv::Vec3f PointCloud::center_of_mass() const {

    cv::Vec3f p0 = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < size(); ++i) {
        cv::Vec3f p = { points.at<float>(i, 0), points.at<float>(i, 1), points.at<float>(i, 2) };
        p0 += p;
    }

    return p0 / size();
}

std::vector<PointAssociation> associate(const PointCloud& source_cloud, const PointCloud& dest_cloud) {
    // TODO: Use KD-Trees for association

    std::vector<PointAssociation> associations;

    const auto& points1 = source_cloud.get_points();
    for (int i = 0; i < source_cloud.size(); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        cv::Vec3f best_match;
        
        cv::Vec3f source_point = { points1.at<float>(i, 0), points1.at<float>(i, 1), points1.at<float>(i, 2) };

        const auto& points2 = dest_cloud.get_points();
        for (int j = 0; j < dest_cloud.size(); ++j) {
            cv::Vec3f dest_point = { points2.at<float>(j, 0), points2.at<float>(j, 1), points2.at<float>(j, 2) };

            float dist = dist_from_origin(source_point - dest_point);
            if(dist < min_dist) {
                min_dist = dist;
                best_match = dest_point;
            }
        }

        associations.push_back(PointAssociation { source_point, best_match });
    }

    return associations;
}

cv::Mat vanilla_icp(const PointCloud& source_cloud, const PointCloud& dest_cloud) {

    // PointCloud source_cleaned = clean_cloud(source_cloud);
    // PointCloud dest_cleaned = clean_cloud(dest_cloud);

    cv::Mat transf = cv::Mat::eye(4, 4, CV_32F);

    for (int i = 0; i < 10; ++i) {

        cv::Vec3f source_mean = source_cloud.center_of_mass();
        cv::Vec3f dest_mean = dest_cloud.center_of_mass();

        std::vector<PointAssociation> associations = associate(source_cloud, dest_cloud);

        cv::Mat H = cv::Mat::zeros(3,3, CV_32F);
        for (const auto& association : associations) {
            cv::Vec3f source_point_shifted = association.source_point - source_mean;
            cv::Vec3f dest_point_shifted = association.dest_point - dest_mean;

            cv::Mat source_point_mat { source_point_shifted };
            cv::Mat dest_point_mat { dest_point_shifted };

            H += (source_point_mat * dest_point_mat.t());
        }

        cv::SVD decomp(H, cv::SVD::FULL_UV);
        cv::Mat R = decomp.vt.t() * decomp.u.t() ;
        cv::Mat T = cv::Mat { dest_mean } - R * cv::Mat { source_mean };
    }

    return transf;
}

// PointCloud clean_cloud(const PointCloud& cloud) {

//     // Remove ground plane
//     PointCloud cloud_ground_removed = remove_ground_plane(cloud);

//     std::vector<PointCloud> clusters;
//     dbscan(cloud_ground_removed, 1.0, 4, clusters);

//     // Remove clusters which contain less points than a given threshold
//     clusters.erase(std::remove_if(clusters.begin(),
//                                 clusters.end(),
//                                 [](const PointCloud& cloud) { return cloud.size() <= 10; }),
//                    clusters.end());

//     PointCloud result;
//     for (const auto& cluster : clusters) {
//         result.add_point(cluster.center_of_mass());
//     }

//     return result;
// }

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

    const cv::Mat points = cloud.get_points();

    for(int i = 0; i < cloud.size(); ++i) {
        ofs << points.at<float>(i, 0) << " " << points.at<float>(i, 1) << " " << points.at<float>(i, 2) << " " << color.x << " " << color.y << " " << color.z << "\n";
    }
}

