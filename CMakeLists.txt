cmake_minimum_required(VERSION 3.26)
project(grf_estimator)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 CONFIG REQUIRED)
list(APPEND CMAKE_PREFIX_PATH /home/savanna-runner/raisim/raisim_workspace/raisimLib/raisim/linux)
find_package(raisim CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories ( ${EIGEN3_INCLUDE_DIRS} )

add_executable(grf_estimator main.cpp)
target_link_libraries(grf_estimator PUBLIC raisim::raisim pthread yaml-cpp)
