#include "point_cloud/point_cloud.h"

PointCloud::PointCloud(const std::ifstream& file) {

}

void PointCloud::read(const std::ifstream& file) { return *this << file; }

void PointCloud::operator<<(const std::ifstream& file) {

}