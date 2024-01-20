cmake_minimum_required(VERSION 3.0)

project(nanoflann_pcl_example)

find_package(nanoflann_pcl 0.1 EXACT REQUIRED)
find_package(nanoflann ${NANOFLANN_VERSION} EXACT REQUIRED PATHS ${NANOFLANN_INSTALL_PREFIX})
find_package(PCL REQUIRED)
find_package(Threads REQUIRED) # may be optional, depending on your system and cmake version

add_executable(nanoflann_pcl_example nanoflann_pcl_example.cpp)
target_include_directories(nanoflann_pcl_example PRIVATE SYSTEM ${PCL_INCLUDE_DIRS})
target_compile_options(nanoflann_pcl_example PRIVATE -Werror -Wall -Wpedantic -Weffc++)
target_link_libraries(nanoflann_pcl_example PRIVATE nanoflann_pcl::nanoflann_pcl nanoflann::nanoflann Threads::Threads ${PCL_LIBRARIES})
