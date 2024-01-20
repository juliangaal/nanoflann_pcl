# `nanoflann_pcl` - [Nanoflann](https://github.com/jlblancoc/nanoflann) adaptor for [PCL](https://pointclouds.org/)

This repo implements a [nanoflann adaptor](https://github.com/jlblancoc/nanoflann/blob/master/examples/pointcloud_adaptor_example.cpp) for efficient n-dimensional neighborhood search with PCL point types.

**Why? Nanoflann is generally faster in robotics contexts than the built-in FLANN adapter for PCL pointclouds that ships with PCL.**

You can expect this speedup, according to the benchmarks on real point clouds (see below):

* insertion: **30%** performance boost
* knn search: **25%** performance boost
* radius search: **24%** performance boost

Existing adapters [1](https://github.com/laboshinl/loam_velodyne/blob/master/include/loam_velodyne/nanoflann_pcl.h), [2](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/master/include/nano_gicp/nanoflann_adaptor.h) considered for [warpsense](https://github.com/juliangaal/warpsense) focus on knn search and either

* **segfault** during radius search
* return **no results** during radius search

This adaptor returns the correct results for both knn and radius search. 

## Usage

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nanoflann_pcl/nanoflann.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// fill cloud ...

// util 
constexpr int k = 100;
constexpr float radius = 10.0;
std::vector<int> nanoflann_indices;
std::vector<float> nanoflann_distances;

// setup
nanoflann::KdTreeFLANN<pcl::PointXYZ> nanoflann_kdtree;
nanoflann_kdtree.setEpsilon(0); // default: 0
nanoflann_kdtree.setSortedResults(true); // default: true
nanoflann_kdtree.setInputCloud(cloud);

// search
nanoflann_kdtree.radiusSearch(cloud->points.back(), radius, nanoflann_indices, nanoflann_distances);
nanoflann_kdtree.nearestKSearch(cloud->points.back(), k, nanoflann_indices, nanoflann_distances);
```

All you need to do is install this header and nanoflann, then replace namespace `pcl::` with `nanoflann::`.

## Installation

2 Options:

* Drop headers into you project and add to your include files
* use provided CMake install target:

  ```bash
    mkdir build && cd build
    cmake .. 
    cmake --build --config Release . 
    ctest -V -R test --output-on-failure # optional: run tests
    cmake --build . --config Release --target install -- -j $(nproc)
  ```
  and see `example/CMakeLists.txt` for how to integrate into a CMake project.

## Supported Nanoflann versions

* 1.5.0 (Released Jun 16, 2023)
* 1.5.1 
* 1.5.2 
* 1.5.3

It makes little sense to support even older versions, because even the upstream apt package installs versions 1.5.3+ (as of 01/24). Furthermore, Nanoflanns only real requirement is a C++11-compatible compiler (or greater) in case you want to simply drop the header file into your project.

If you are dependent on earlier versions of Nanoflann, feel free to open a PR. Note that pre 1.5.0 has a different API for radius search, explained [here ('API changes')](https://github.com/jlblancoc/nanoflann/releases/tag/v1.5.0)

## Benchmarks

Benchmarks where performed using [Catch2 v3](https://github.com/catchorg/Catch2/blob/devel/docs/benchmarks.md) on a modest Intel i5-4670 @ 3.40GHz × 4.

Notation:

* point cloud sizes: denoted by Nk (N thousand points)
* number of neighbors searched: denoted by Nn (N neighbors)
* size of radius searched: denoted by Nm (N meters)
* ms = milliseconds
* Std = standard deviation across n iterations

### Synthetic

Pointclouds filled with random points. May or not be realistic for your usage scenario.

#### Insertion

Pointcloud: 1k - 100mio points. Iterations: 1k.

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| tree insertion 1k | 0.09820 | **0.05533** | **0.00165** | 0.00272 |
| tree insertion 10k | 1.31673 | **0.84244** | 0.00695 | **0.00434** |
| tree insertion 100k | 18.74870 | **12.52820** | 0.06198 | **0.02963** |
| tree insertion 1mio | 463.73200 | **304.11100** | **4.17225** | 14.31570 |
| tree insertion 10mio | 10045.50000 | **6094.39000** | 50.90730 | **35.52600** |
| tree insertion 100mio | 184126.00000 | **110075.00000** | 3621.70000 | **1174.57000** |

#### KNN Search 

Pointcloud: 100k points. Iterations: 1k. Tested: 1 - 1000 neighbors

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| knn search 1 | 0.00132 | **0.00012** | 0.00005 | 0.00000 |
| knn search 10 | 0.00166 | **0.00036** | 0.00014 | **0.00001** |
| knn search 100 | 0.00514 | **0.00354** | 0.00027 | **0.00015** |
| knn search 1000 | **0.10350** | 0.27315 | **0.00309** | 0.00682 |

#### Radius Search

Pointcloud: 100k points. Iterations: 1k. 1m - 50m radius.

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| Radius search 1m  | **0.00137** | 0.00014 | 0.00009 | **0.00003** |
| Radius search 10m | 0.00142 | **0.00014** | 0.00015 | **0.00001** |
| Radius search 20m | 0.00141 | **0.00014** | 0.00005 | **0.00000** |
| Radius search 30m | 0.00155 | **0.00027** | 0.00007 | **0.00001** |
| Radius search 40m | 0.00156 | **0.00027** | 0.00010 | **0.00002** |
| Radius search 50m | 0.00157 | **0.00027** | 0.00014 | **0.00001** |

### Real-World pointclouds 

`benchmark/data` includes multiple scans from 

* Velodyne VLP32E in Hanover, provided by [University of Osnabrueck](http://kos.informatik.uni-osnabrueck.de/3Dscans/)
* Riegl VLZ terrestrial scanner scans, also provided by ^^
* Bunny, provided by [Standford](https://graphics.stanford.edu/data/3Dscanrep/)

#### Insertion

Iterations: 10

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| cloud_0601 3.7k | 0.41748 | **0.31421** | **0.00711** | 0.01105 |
| cloud_0422 9.7k | 1.06278 | **0.77641** | 0.04849 | **0.01556** |
| cloud_0043 13k | 1.48485 | **1.10457** | 0.07269 | **0.02060** |
| cloud_0368 18.8k | 2.12193 | **1.60147** | 0.02214 | **0.01090** |
| carpark 52k | 7.44440 | **5.85080** | 0.09779 | **0.04552** |
| bunny 40k | 4.25128 | **3.21900** | 0.03517 | **0.00950** |
| scan_006 560k | 93.51190 | **74.09750** | **1.39890** | 1.48065 |
| scan_007 825k | 154.64700 | **118.64000** | 1.93568 | **1.33667** |


#### KNN Search 

Iterations: 1k

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| cloud_0601 3.7k, 100n | 0.00710 | **0.00685** | 0.00151 | **0.00081** |
| cloud_0422 9.7k, 100n | 0.00389 | **0.00315** | 0.00073 | **0.00029** |
| cloud_0043 13k, 100n | 0.00646 | **0.00485** | 0.00129 | **0.00055** |
| cloud_0368 18.8k, 100n | 0.00505 | **0.00442** | 0.00042 | **0.00033** |
| bunny 40k, 100n | 0.00827 | **0.00509** | 0.00107 | **0.00081** |
| carpark 52k, 100n | 0.00464 | **0.00353** | **0.00063** | 0.00083 |
| scan_006 560k, 100n | 0.00673 | **0.00627** | **0.00127** | 0.00139 |
| scan_007 825k, 100n | 0.00516 | **0.00455** | **0.00081** | 0.00091 |


#### Radius Search

Iterations: 1k

| Test Case | Flann Mean (ms) | Nanoflann Mean (ms) | Flann Std (ms) | Nanoflann Std (ms) |
| --- | --- | --- | --- | --- |
| cloud_0601 3.7k, 10m | 0.01555 | **0.00930** | 0.00219 | **0.00116** |
| cloud_0422 9.7k, 10m | 0.39186 | **0.29293** | 0.03156 | **0.01990** |
| cloud_0043 13k, 10m | 0.52155 | **0.41522** | 0.02084 | **0.02042** |
| cloud_0368 18.8k, 10m | 0.69870 | **0.55135** | 0.03031 | **0.02405** |
| bunny 40k, 10m | 2.35269 | **1.83126** | **0.07192** | 0.07355 |
| carpark 52k, 10m | 2.61272 | **2.02952** | 0.14101 | **0.06963** |
| scan_006 560k, 10m | 2.88219 | **2.37084** | **0.10212** | 0.11535 |
| scan_007 825k, 10m | 2.45282 | **1.96704** | **0.09085** | 0.10605 |


### Reproduce Benchmarks 

* Run benchmark. For a list of benchmarks, see `ctest -N` and chose e.g. `ctest -V -R benchmark_insertion`. Catch2 will save `NAME_OF_BENCHMARK.xml`
* Process `NAME_OF_BENCHMARK.xml` with `catch2_postprocessing.py` in `util/`
* e.g. `python3 catch2_postprocessing.py benchmark_insertion.xml --categories "flann tree insertion" "nanoflann tree insertion"`
* if required, save csv file with `--export_csv`
