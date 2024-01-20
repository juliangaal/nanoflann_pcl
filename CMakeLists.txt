cmake_minimum_required(VERSION 3.0)

project(nanoflann_pcl VERSION 0.1
                      DESCRIPTION "A header-only c++ library that provides a Nanoflann adapter for Point Cloud Library (PCL) types"
                      HOMEPAGE_URL "https://github.com/juliangaal/nanoflann_pcl")

include(cmake/benchmark.cmake)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(NANOFLANN_VERSION "1.5.0" CACHE STRING "Nanoflann version")
set(CMAKE_INSTALL_PREFIX "/usr/local/" CACHE STRING "install prefix")
option(BUILD_TESTS OFF)

add_compile_definitions(DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")


find_package(PCL REQUIRED)
find_package(nanoflann ${NANOFLANN_VERSION} EXACT REQUIRED PATHS ${NANOFLANN_INSTALL_PREFIX})

if (BUILD_TESTS)
  enable_testing()
  find_package(Catch2 3 REQUIRED)
  include_directories(include util)
  include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
  add_compile_options(-Werror -Wall -Wpedantic -Weffc++)
  link_libraries(Catch2::Catch2WithMain nanoflann::nanoflann ${PCL_LIBRARIES})
  
  # general tests
  add_executable(test_nanoflann_pcl test/test_nanoflann_pcl.cpp)
  
  # synthetic benchmarks
  add_executable(synth_benchmark_insertion benchmark/synth/benchmark_insertion.cpp)
  add_executable(synth_benchmark_knn_search benchmark/synth/benchmark_knn_search.cpp)
  add_executable(synth_benchmark_radius_search benchmark/synth/benchmark_radius_search.cpp)
  
  # real benchmarks
  add_executable(real_benchmark_insertion benchmark/real/benchmark_insertion.cpp)
  add_executable(real_benchmark_knn_search benchmark/real/benchmark_knn_search.cpp)
  add_executable(real_benchmark_radius_search benchmark/real/benchmark_radius_search.cpp)
  
  # test targets
  add_test(NAME test COMMAND test_nanoflann_pcl)
  
  # benchmarks
  add_benchmark(synth_benchmark_insertion 10)
  add_benchmark(synth_benchmark_radius_search 1000)
  add_benchmark(synth_benchmark_knn_search 1000)
  add_benchmark(real_benchmark_insertion 10)
  add_benchmark(real_benchmark_radius_search 1000)
  add_benchmark(real_benchmark_knn_search 1000)
endif()

# support install targets. Thank you to the eye opening
# https://github.com/bernedom/SI/blob/main/cmake/SIConfig.cmake.in
# and accompaning https://dominikberner.ch/cmake-interface-lib/
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(nanoflann_pcl INTERFACE)

target_compile_features(nanoflann_pcl INTERFACE cxx_std_11)

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/SI/cmake)

install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)


