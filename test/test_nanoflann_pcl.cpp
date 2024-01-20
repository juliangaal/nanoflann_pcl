#include "util.h"
#include <catch2/catch_test_macros.hpp>
#include <nanoflann.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

constexpr int n = 1'000;
constexpr int k = 100;
constexpr float radius = 100;
auto cloud = generate_cloud<pcl::PointXYZ>(n);

TEST_CASE("Radius search", "[radius]") {
    // flann test (radius)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    flann_kdtree.radiusSearch(cloud->points.back(), radius, flann_indices, flann_distances);

    // nanoflann test (radius)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);

    REQUIRE(flann_indices.size() == nanoflann_indices.size());
    REQUIRE(flann_distances.size() == nanoflann_distances.size());
    REQUIRE(flann_distances.size() == flann_indices.size());

    for (auto i = 0u; i < flann_indices.size(); ++i) {
        REQUIRE(flann_indices[i] == nanoflann_indices[i]);
    }
}

TEST_CASE("knn search", "[knn]") {
    // flann test (knn)
    std::vector<int> flann_indices;
    std::vector<float> flann_distances;
    pcl::KdTreeFLANN<pcl::PointXYZ> flann_kdtree;
    flann_kdtree.setSortedResults(true);
    flann_kdtree.setInputCloud(cloud);
    flann_kdtree.nearestKSearch(cloud->points.back(), k, flann_indices, flann_distances);

    // nanoflann test (knn)
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);

    REQUIRE(flann_indices.size() == nanoflann_indices.size());
    REQUIRE(flann_distances.size() == nanoflann_distances.size());
    REQUIRE(flann_distances.size() == flann_indices.size());

    for (auto i = 0u; i < flann_indices.size(); ++i) {
        REQUIRE(flann_indices[i] == nanoflann_indices[i]);
    }
}
