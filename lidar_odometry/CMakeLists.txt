cmake_minimum_required(VERSION 3.0.0)

project(lidar_odometry VERSION 0.1.0)

add_library(${PROJECT_NAME}
    # include/lidar_odometry/
    include/clustering/dbscan.h
    include/point_cloud/point_cloud.h
    src/dbscan.cpp
    src/point_cloud.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} 
    PUBLIC ${OpenCV_LIBS}
)