#pragma once

#include <iostream>
#include <string>
#include <vector>


struct Point {
    float x;
    float y;
    float z;
};


class PointCloud {
public:
    PointCloud() = default;
    PointCloud(const std::ifstream& file);

    void read(const std::ifstream& file);

    void operator<<(const std::ifstream& file);

private:
    std::vector<Point> points;
};