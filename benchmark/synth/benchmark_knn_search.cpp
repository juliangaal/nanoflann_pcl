#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

constexpr int n = 100'000;
auto cloud = generate_cloud<pcl::PointXYZ>(n);

TEST_CASE("knn search 1n", "[knn]") {
    constexpr int k = 1;
    
    // flann benchmarks (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(cloud->points.back(), k, flann_indices, flann_distances);
    };

    // nanoflann benchmarks (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("knn search 10n", "[knn]") {
    constexpr int k = 10;
    
    // flann benchmarks (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(cloud->points.back(), k, flann_indices, flann_distances);
    };

    // nanoflann benchmarks (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("knn search 100n", "[knn]") {
    constexpr int k = 100;
    
    // flann benchmarks (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(cloud->points.back(), k, flann_indices, flann_distances);
    };

    // nanoflann benchmarks (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);
    };
}

TEST_CASE("knn search 1000n", "[knn]") {
    constexpr int k = 1000;
    
    // flann benchmarks (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(cloud->points.back(), k, flann_indices, flann_distances);
    };

    // nanoflann benchmarks (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);
    };
}
