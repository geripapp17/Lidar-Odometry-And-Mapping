#include "point_cloud/point_cloud.h"

int main(int argc, char** argv) {

    PointCloud map, prev_cloud;
    cv::Mat body_to_body0 = cv::Mat::eye(4, 4, CV_32F);

    for (int i = 20; i < 50; ++i) {

        std::string path = "/home/geri/work/c++/Lidar-Odometry/data/default_path/test_fn" + std::to_string(i) + ".xyz";

        std::ifstream ifs{ path, std::ifstream::in };
        if(!ifs.is_open()) { return 1; }

        PointCloud cur_cloud{ ifs };
        ifs.close();

        if (!prev_cloud.empty()) {
            cv::Mat T = vanilla_icp(prev_cloud, cur_cloud);

            body_to_body0 = T * body_to_body0;

            const auto& cur_cloud_points = cur_cloud.get_points();
            for (int i = 0; i < cur_cloud.size(); ++i) {
                cv::Vec4f point { cur_cloud_points.at<float>(i, 0), cur_cloud_points.at<float>(i, 1), cur_cloud_points.at<float>(i, 2), 1.0f };
                cv::Mat point_transformed  = body_to_body0 * cv::Mat { point };
                point_transformed /= point_transformed.at<float>(3);
                map.add_point( point_transformed );
            }
        }

        prev_cloud = cur_cloud;

        std::ofstream ofs{ "/home/geri/work/c++/Lidar-Odometry/output/map" + std::to_string(i) + ".ply", std::ofstream::out };
        write_ply(ofs, map, { (rand() + 100) % 256, (rand() + 100) % 256, (rand() + 100) % 256 });
        ofs.close();
    }

    return 0;
}

// https://github.com/ClayFlannigan/icp/blob/master/icp.py