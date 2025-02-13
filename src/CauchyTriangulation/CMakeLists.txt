find_package(Eigen3 REQUIRED NO_MODULE)

add_library(CauchyTriangulation SHARED CauchyTriangulation.cpp CauchyTriangulation.h)
target_include_directories(CauchyTriangulation INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CauchyTriangulation gtsam Eigen3::Eigen)