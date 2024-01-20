#pragma once

#include <memory>
#include <nanoflann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static_assert(NANOFLANN_VERSION == 0x150 || NANOFLANN_VERSION == 0x151 || NANOFLANN_VERSION == 0x152 ||
                  NANOFLANN_VERSION == 0x153,
              "Unsupported nanoflann version. Supported are version 1.5.0-1.5.3");

namespace nanoflann {

template <typename PointT> class KdTreeFLANN {
  public:
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    using PointCloudConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

    using Ptr = std::shared_ptr<KdTreeFLANN<PointT>>;
    using ConstPtr = std::shared_ptr<const KdTreeFLANN<PointT>>;
    using IndicesPtr = std::shared_ptr<std::vector<int>>;
    using IndicesConstPtr = std::shared_ptr<const std::vector<int>>;

    KdTreeFLANN(bool sorted = false);

    ~KdTreeFLANN() = default;

    inline void setEpsilon(float eps) { _params.eps = eps; }

    inline void setSortedResults(bool sorted) { _params.sorted = sorted; }

    void setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr());

    inline PointCloudConstPtr getInputCloud() const { return _adaptor.pcl; }

    int nearestKSearch(const PointT& point, int k, std::vector<int>& k_indices,
                       std::vector<float>& k_sqr_distances) const;

    int radiusSearch(const PointT& point, float radius, std::vector<int>& k_indices,
                     std::vector<float>& k_sqr_distances) const;

  protected:
    nanoflann::SearchParameters _params;

    struct PointCloud_Adaptor {
        PointCloud_Adaptor() : pcl(), indices() {}
        ~PointCloud_Adaptor() = default;
        inline size_t kdtree_get_point_count() const;
        inline float kdtree_get_pt(const size_t idx, int dim) const;
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
        PointCloudConstPtr pcl;
        IndicesConstPtr indices;
    };

    using KDTreeFlann_PCL_SO3 = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::SO3_Adaptor<float, PointCloud_Adaptor>,
                                                                    PointCloud_Adaptor, 3, int>;

    PointCloud_Adaptor _adaptor;

    KDTreeFlann_PCL_SO3 _kdtree;
};

template <typename PointT>
inline KdTreeFLANN<PointT>::KdTreeFLANN(bool sorted)
    : _params(), _adaptor(), _kdtree(3, _adaptor, KDTreeSingleIndexAdaptorParams(25)) {
    _params.sorted = sorted;
}

template <typename PointT>
inline void KdTreeFLANN<PointT>::setInputCloud(const KdTreeFLANN::PointCloudConstPtr& cloud,
                                               const IndicesConstPtr& indices) {
    _adaptor.pcl = cloud;
    _adaptor.indices = indices;
    _kdtree.buildIndex();
}

template <typename PointT>
inline int KdTreeFLANN<PointT>::nearestKSearch(const PointT& point, int num_closest, std::vector<int>& k_indices,
                                               std::vector<float>& k_sqr_distances) const {
    k_indices.resize(num_closest);
    k_sqr_distances.resize(num_closest);

    nanoflann::KNNResultSet<float, int> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    _kdtree.findNeighbors(resultSet, point.data, _params);
    return resultSet.size();
}

template <typename PointT>
inline int KdTreeFLANN<PointT>::radiusSearch(const PointT& point, float radius, std::vector<int>& k_indices,
                                             std::vector<float>& k_sqr_distances) const {
    static std::vector<nanoflann::ResultItem<int, float>> result;
    result.reserve(128);

    // From nanoflann README
    // > Important note: If L2 norms are used, notice that search radius and all passed and returned      distances are
    // actually squared distances.
    //
    // I argue that the user expects searches in m. Therefore square the radius for the user.
    //
    // Earlier versions of this lbrary added tiny offset to effectively search <= radius, but
    // this does not correspond to the behavior in pcl, therfore remove offset
    float sq_radius = radius * radius; // + 0.05

    const size_t n_found = _kdtree.radiusSearch(point.data, sq_radius, result);

    k_indices.resize(n_found);
    k_sqr_distances.resize(n_found);
    for (size_t i = 0; i < n_found; i++) {
        k_indices[i] = result[i].first;
        k_sqr_distances[i] = result[i].second;
    }
    return n_found;
}

template <typename PointT> inline size_t KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_point_count() const {
    if (indices) {
        return indices->size();
    }
    if (pcl) {
        return pcl->points.size();
    }
    return 0;
}

template <typename PointT>
inline float KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_pt(const size_t idx, int dim) const {
    const PointT& p = (indices) ? pcl->points[(*indices)[idx]] : pcl->points[idx];
    if (dim == 0) {
        return p.x;
    } else if (dim == 1) {
        return p.y;
    } else {
        return p.z;
    }

    return 0.0;
}

} // namespace nanoflann
