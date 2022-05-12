#include "point_cloud/point_cloud.h"

int main(int argc, char** argv) {

    PointCloud map;
    Transformation body_to_body0;
    PointCloud prev_cloud;

    for (int i = 10; i < 50; ++i) {

        std::string path = "/home/geri/work/c++/Lidar-Odometry/data/default_path/test_fn" + std::to_string(i) + ".xyz";

        std::ifstream ifs{ path, std::ifstream::in };
        if(!ifs.is_open()) { return 1; }

        PointCloud cur_cloud{ ifs };
        ifs.close();

        if (!prev_cloud.empty()) {
            Transformation RT = vanilla_icp(prev_cloud, cur_cloud);

            body_to_body0.R = RT.R * body_to_body0.R;
            body_to_body0.T = RT.R * body_to_body0.T + RT.T;

            for (auto& point : cur_cloud) {
                cv::Mat point_transformed  = body_to_body0.R * cv::Mat { point } + body_to_body0.T;
                map.add_point(cv::Point3f { point_transformed.at<float>(0, 0), point_transformed.at<float>(1, 0), point_transformed.at<float>(2, 0) });
            }
        }
        else {
            for (auto& point : cur_cloud) {
                map.add_point(cv::Point3f { point.x, point.y, point.z });
            }
            body_to_body0.R = cv::Mat::eye(3, 3, CV_32F);
            body_to_body0.T = cv::Mat::eye(3, 1, CV_32F);
        }

        prev_cloud = cur_cloud;

        std::ofstream ofs{ "/home/geri/work/c++/Lidar-Odometry/output/map" + std::to_string(i) + ".ply", std::ofstream::out };
        write_ply(ofs, map);
        ofs.close();
    }

    return 0;
}