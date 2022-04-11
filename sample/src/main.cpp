#include "point_cloud/point_cloud.h"
#include "clustering/dbscan.h"

int main(int argc, char** argv) {

    std::ifstream ifs{ "/home/geri/work/c++/Lidar-Odometry/data/allee_parking/test_fn1.xyz", std::ifstream::in };
    if(!ifs.is_open()) { return 1; }

    PointCloud cloud{ ifs };
    ifs.close();

    std::vector<PointCloud> clusters;
    dbscan(cloud, 1.0, 4, clusters);


    // std::ofstream ofs{ ".output/out1.ply", std::ofstream::out };
    // cloud.write(ofs);
    // cloud.print();

    // ofs.close();

    return 0;
}