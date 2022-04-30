#include "point_cloud/point_cloud.h"
#include "clustering/dbscan.h"

int main(int argc, char** argv) {

    std::ifstream ifs{ "/home/geri/work/c++/Lidar-Odometry/data/default_path/test_fn1.xyz", std::ifstream::in };
    if(!ifs.is_open()) { return 1; }

    PointCloud cloud{ ifs };
    ifs.close();

    // Remove ground plane
    remove_ground_plane(cloud);

    std::vector<PointCloud> clusters;
    dbscan(cloud, 1.0, 4, clusters);

    // Remove clusters which contain less points than a given threshold
    clusters.erase(std::remove_if(clusters.begin(),
                                  clusters.end(),
                                  [](const PointCloud& cloud) { return cloud.size() <= 10; }),
                   clusters.end());

    std::cout << clusters.size() << std::endl;

    std::ofstream ofs{ "/home/geri/work/c++/Lidar-Odometry/output/without_ground.ply", std::ofstream::out };
    write_ply(ofs, clusters);
    ofs.close();

    return 0;
}