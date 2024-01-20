#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace fs = std::filesystem;
constexpr int k = 100;

TEST_CASE("cloud_0601 3.7k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0601.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0422 9.7k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0422.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0043 13k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0043.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0368 18.8k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0368.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("bunny 40k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "bun000_UnStructured.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("carpark 52k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "carpark.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("scan_006 560k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_006.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}

TEST_CASE("scan_007 825k, 100n", "[knn]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_007.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann knn search") {
        return flann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };

    // nanoflann benchmark (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann knn search") {
        return nanoflann_kdtree.nearestKSearch(query, k, flann_indices, flann_distances);
    };
}
