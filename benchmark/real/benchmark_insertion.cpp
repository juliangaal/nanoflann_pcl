#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace fs = std::filesystem;

TEST_CASE("cloud_0601 3.7k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0601.pcd").string());
    
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

TEST_CASE("cloud_0422 9.7k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0422.pcd").string());
    
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

TEST_CASE("cloud_0043 13k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0043.pcd").string());
    
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

TEST_CASE("cloud_0368 18.8k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0368.pcd").string());
    
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

TEST_CASE("carpark 52k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "carpark.pcd").string());
    
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

TEST_CASE("bunny 40k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "bun000_UnStructured.pcd").string());
    
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

TEST_CASE("scan_006 560k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_006.pcd").string());
    
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

TEST_CASE("scan_007 825k", "[insertion]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_007.pcd").string());
    std::cout << cloud->size() << "\n";
    
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
