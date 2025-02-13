add_library(data_loader SHARED data_loader.h data_loader.cpp)
target_include_directories(data_loader INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(data_loader gtsam)