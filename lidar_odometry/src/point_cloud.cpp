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

cv::Vec3f PointCloud::center_of_mass() const {

    cv::Vec3f p0 = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < size(); ++i) {
        cv::Vec3f p = { points.at<float>(i, 0), points.at<float>(i, 1), points.at<float>(i, 2) };
        p0 += p;
    }

    return p0 / size();
}

void PointCloud::transform_cloud(const cv::Mat& T) {

    cv::Mat tmp = points.clone().t();
    tmp.push_back( cv::Mat::ones(1, tmp.cols, CV_32F) );
    cv::Mat tmp2 = T * tmp;
    tmp2.pop_back();
    cv::transpose(tmp2, tmp2);

    points = tmp2.clone();
}

void remove_ground_plane(std::unordered_set<cv::Point3f>& points) {

    srand(time(0));

    std::vector<cv::Point3f> point_vec;
    for(const auto& point : points) {
        point_vec.push_back(point);
    }

    // Select points that are part of the ground plane.
    std::vector<cv::Point3f> ground_plane = ransac(point_vec);

    // Remove ground points from the points which were possible ground points.
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

cv::Mat associate(const PointCloud& source_cloud, const PointCloud& dest_cloud) {

    cv::Mat associated_dest;

    const auto& points1 = source_cloud.get_points();
    const auto& points2 = dest_cloud.get_points();

    for (int i = 0; i < source_cloud.size(); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        cv::Mat best_match;
        
        cv::Vec3f p1 { points1.at<float>(i, 0), points1.at<float>(i, 1), points1.at<float>(i, 2) };
        cv::Mat source_point { p1 };

        for (int j = 0; j < dest_cloud.size(); ++j) {
            cv::Vec3f p2 { points2.at<float>(j, 0), points2.at<float>(j, 1), points2.at<float>(j, 2) };
            cv::Mat dest_point { p2 };

            float dist = point_to_point_distance(p1, p2);
            if(dist < min_dist) {
                min_dist = dist;
                best_match = dest_point.clone();
            }
        }

        associated_dest.push_back( best_match.t() );
    }

    return associated_dest;
}

cv::Mat vanilla_icp(const PointCloud& dest_cloud, PointCloud source_cloud) {

    // PointCloud source_cleaned = clean_cloud(source_cloud);
    // PointCloud dest_cleaned = clean_cloud(dest_cloud);

    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);     ///< Final transformation between the current and previous clouds.

    for (int i = 0; i < 25; ++i) {
        cv::Vec3f src_mean = source_cloud.center_of_mass();
        cv::Vec3f dst_mean = dest_cloud.center_of_mass();

        auto& source_cloud_mat = source_cloud.get_points();
        cv::Mat src;
        for(int i = 0; i < source_cloud.size(); ++i) {
            src.push_back( (source_cloud_mat.row(i).colRange(0, 3) - cv::Mat { src_mean }.t()) );
        }

        cv::Mat associated_dest = associate(source_cloud, dest_cloud);

        cv::Mat dst;
        for(int i = 0; i < associated_dest.rows; ++i) {
            dst.push_back( associated_dest.row(i).colRange(0, 3) - cv::Mat { dst_mean }.t() );
        }

        cv::transpose(src, src);
        cv::Mat H = src * dst;

        cv::SVD decomp(H, cv::SVD::FULL_UV);
        cv::Mat R = decomp.vt.t() * decomp.u.t() ;
        cv::Mat t = cv::Mat { dst_mean } - R * cv::Mat { src_mean };

        cv::Mat T_new = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat aux_R = T_new.rowRange(0, 3).colRange(0, 3);
        cv::Mat aux_t = T_new.rowRange(0, 3).col(3);

        R.copyTo(aux_R);
        t.copyTo(aux_t);

        T = T * T_new;
        source_cloud.transform_cloud(T);
    }

    return T;
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

void write_ply(const std::string path, const cv::Mat& cloud, const cv::Point3i color) {

    int prev_cnt = 0;
    std::ifstream ifs;

    std::string out_path = path + "map.ply";
    std::string new_path = path + "tmp.ply";

    if(file_exists(path + "map.ply")) {
        std::rename(out_path.c_str(), new_path.c_str());
        ifs.open(path + "tmp.ply");

        for (int i = 0; i < 10; ++i) {
            std::string line;
            std::getline(ifs, line);

            if(2 == i) {
                std::string tmp;
                std::istringstream iss{ line };
                iss >> tmp >> tmp >> tmp;
                prev_cnt = stoi(tmp);
            }
        }
    }

    std::ofstream ofs{ out_path, std::ofstream::out };

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex "<< prev_cnt + cloud.rows << std::endl;
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    if(0 != prev_cnt) {
        std::string line;
        while(std::getline(ifs, line)) {
            ofs << line << "\n";
        }

        std::remove(new_path.c_str());
    }

    for(int i = 0; i < cloud.rows; ++i) {
        ofs << cloud.at<float>(i, 0) << " " << cloud.at<float>(i, 1) << " " << cloud.at<float>(i, 2) << " " << color.x << " " << color.y << " " << color.z << "\n";
    }

    ofs.close();
}

bool file_exists(const std::string path) {

    std::ifstream file { path };
    if(file) {
        file.close();
        return true;
    }

    file.close();
    return false;
}