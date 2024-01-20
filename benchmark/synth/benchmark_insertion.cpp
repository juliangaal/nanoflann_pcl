#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

TEST_CASE("tree insertion 1k", "[insertion]") {
    constexpr int n = 1'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}

TEST_CASE("tree insertion 10k", "[insertion]") {
    constexpr int n = 10'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}

TEST_CASE("tree insertion 100k", "[insertion]") {
    constexpr int n = 100'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}

TEST_CASE("tree insertion 1mio", "[insertion]") {
    constexpr int n = 1'000'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}

TEST_CASE("tree insertion 10mio", "[insertion]") {
    constexpr int n = 10'000'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}

TEST_CASE("tree insertion 100mio", "[insertion]") {
    constexpr int n = 100'000'000;
    auto cloud = generate_cloud<pcl::PointXYZ>(n);
    
    // flann benchmark
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    BENCHMARK("flann tree insertion") { return flann_kdtree.setInputCloud(cloud); };

    // nanoflann benchmark
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    BENCHMARK("nanoflann tree insertion") { return nanoflann_kdtree.setInputCloud(cloud); };
}
