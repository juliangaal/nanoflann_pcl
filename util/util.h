#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <stdexcept>
#include <filesystem>

template <typename T>
T get_random_number(T min, T max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

template <typename T> 
typename pcl::PointCloud<T>::Ptr generate_cloud(int n, float min = -1000.0, float max = 1000.0) {
    typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    cloud->reserve(n);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);

    for (int i = 0; i < n; ++i) {
        pcl::PointXYZ point;
        point.x = dis(gen);
        point.y = dis(gen);
        point.z = dis(gen);
        cloud->push_back(point);
    }

    return cloud;
}

template <typename T> 
typename pcl::PointCloud<T>::Ptr read_cloud(std::string filepath) {
    typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    if (pcl::io::loadPCDFile<T>(filepath, *cloud) == -1) {
        throw std::runtime_error("Couldn't read file.\n");
    }
    return cloud;
}
