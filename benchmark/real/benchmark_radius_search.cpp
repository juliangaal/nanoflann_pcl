#include "util.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace fs = std::filesystem;
constexpr float radius = 10.0;

TEST_CASE("cloud_0601 3.7k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0601.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0422 9.7k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0422.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0043 13k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0043.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("cloud_0368 18.8k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "cloud_0368.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("bunny 40k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "bun000_UnStructured.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("carpark 52k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "carpark.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("scan_006 560k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_006.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}

TEST_CASE("scan_007 825k, 10m", "[radius]") {
    auto cloud = read_cloud<pcl::PointXYZ>((fs::path(DATA_DIR) / "scan_007.pcd").string());
    auto query = cloud->points.at(get_random_number<size_t>(0, cloud->points.size()-1));

    // flann benchmark (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    BENCHMARK("flann radius search") {
        return flann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };

    // nanoflann benchmark (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    BENCHMARK("nanoflann radius search") {
        return nanoflann_kdtree.radiusSearch(query, radius, flann_indices, flann_distances);
    };
}
