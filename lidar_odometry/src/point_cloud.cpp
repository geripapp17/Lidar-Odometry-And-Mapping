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

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex "<< cloud.points.size() << std::endl;
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    for(const auto& point : cloud.points) {
        // ofs << point->x << " " << point->y << " " << point->z << " " << point->color.r << " " << point->color.g << " " << point->color.b << std::endl;
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
