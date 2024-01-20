#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

constexpr int n = 100'000;
auto cloud = generate_cloud<pcl::PointXYZ>(n, -50, 50);
    
TEST_CASE("Radius search 1m", "[radius]") {
    constexpr float radius = 1;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("Radius search 10m", "[radius]") {
    constexpr float radius = 10;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("Radius search 20m", "[radius]") {
    constexpr float radius = 20;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("Radius search 30m", "[radius]") {
    constexpr float radius = 30;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("Radius search 40m", "[radius]") {
    constexpr float radius = 40;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("Radius search 50m", "[radius]") {
    constexpr float radius = 50;

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
    };
}

