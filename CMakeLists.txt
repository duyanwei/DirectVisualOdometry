cmake_minimum_required(VERSION 3.5)
project(dvo)

# version
set(DVO_VERSION_MAJOR 0)
set(DVO_VERSION_MINOR 1)

# package
find_package(GTSAM REQUIRED PATHS "/home/duyanwei/duyanwei/gtsam/build")
find_package(OpenCV REQUIRED PATHS "/home/duyanwei/duyanwei/opencv/build")

# headers
include_directories(include)

add_library(${PROJECT_NAME} SHARED src/direct_method_factor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${GTSAM_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} gtsam gtsam_unstable ${OPENCV_LIBS})