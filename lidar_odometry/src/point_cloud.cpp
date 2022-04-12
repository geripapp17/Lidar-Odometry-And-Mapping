#include "point_cloud/point_cloud.h"

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