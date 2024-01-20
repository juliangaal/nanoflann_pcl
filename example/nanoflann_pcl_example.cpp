#include <nanoflann.hpp>
#include <nanoflann_pcl/nanoflann.hpp>
#include <random>

int main(void) {
    int n = 1'000;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(n);

    // random data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1000.0, 1000.0);

    for (int i = 0; i < n; ++i) {
        pcl::PointXYZ point;
        point.x = dis(gen); 
        point.y = dis(gen); 
        point.z = dis(gen); 
        cloud->push_back(point);
    }


    // usage
    std::vector<int> nanoflann_indices;
    std::vector<float> nanoflann_distances;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
    nanoflann_kdtree.setInputCloud(cloud);
    nanoflann_kdtree.radiusSearch(cloud->points.back(), 100, nanoflann_indices, nanoflann_distances);
    nanoflann_kdtree.nearestKSearch(cloud->points.back(), 100, nanoflann_indices, nanoflann_distances);
}
