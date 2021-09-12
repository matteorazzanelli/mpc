project(mpc_osqp)

cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 20)

# Options
option(BUILD_TESTS "Want to build tests?" ON)

# Dependency management
include(cmake/setupDependencies.cmake)
include(FetchContent)

# Setup dependencies
find_package(Eigen3)
setupDependencies(
    TARGET ${PROJECT_NAME}
    LIBS osqp gtest
    TESTS ${BUILD_TESTS}
)

