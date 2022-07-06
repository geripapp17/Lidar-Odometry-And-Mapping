#include "point_cloud/point_cloud.h"

int main(int argc, char** argv) {

    PointCloud prev_cloud;
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);

    for (int i = 20; i < 50; ++i) {

        std::string path = "/home/geri/work/c++/Lidar-Odometry/data/default_path/test_fn" + std::to_string(i) + ".xyz";

        std::ifstream ifs{ path, std::ifstream::in };
        if(!ifs.is_open()) { return 1; }

        PointCloud cur_cloud{ ifs };
        ifs.close();

        write_ply("/home/geri/work/c++/Lidar-Odometry/output/", cur_cloud.get_points(), { (rand() + 100) % 256, (rand() + 100) % 256, (rand() + 100) % 256 });

        if (!prev_cloud.empty()) {
            cv::Mat T_new = vanilla_icp(prev_cloud, cur_cloud);
            T = T * T_new;

            cv::Mat cloud = cur_cloud.get_points().t();
            cloud.push_back(cv::Mat::ones(1, cloud.cols, CV_32F));

            cv::Mat cloud_transformed = T * cloud;
            cloud_transformed.pop_back();
            cv::transpose(cloud_transformed, cloud_transformed);

            write_ply("/home/geri/work/c++/Lidar-Odometry/output/", cloud_transformed, { (rand() + 100) % 256, (rand() + 100) % 256, (rand() + 100) % 256 });
        }

        prev_cloud = cur_cloud;
    }

    return 0;
}