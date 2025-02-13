cmake_minimum_required(VERSION 3.10.0)
project(CauchyTriangulation)

if (CMAKE_BUILD_TYPE MATCHES Release)
    set(GTSAM_DIR "/home/padossa/bin/gtsam")
else()
    set(GTSAM_DIR "/home/padossa/bin/gtsam-debug")
endif()

find_package(GTSAM REQUIRED PATHS ${GTSAM_DIR})

add_executable(scratch scratch.cpp)
target_link_libraries(scratch gtsam)

add_subdirectory(cauchy_triangulation)
add_subdirectory(CauchyTriangulation)
add_subdirectory(data_loader)

add_executable(TriangulationCauchyExample TriangulationCauchyExample.cpp)
target_link_libraries(TriangulationCauchyExample gtsam CauchyTriangulation)
target_include_directories(TriangulationCauchyExample PUBLIC CauchyTriangulation)

add_executable(paper_real_data paper_real_data.cpp)
target_link_libraries(paper_real_data gtsam cauchy_triangulation data_loader)
target_include_directories(paper_real_data PUBLIC cauchy_triangulation data_loader)