project(mpc_osqp)

cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 20)

# Options
option(BUILD_TESTS "Want to build tests?" ON)

# Dependency management
include(cmake/setup.cmake)
include(FetchContent)

# Setup dependencies and flags
list(APPEND LIBS
    Eigen3
    osqp
    gtest
)
find_package(Eigen3 REQUIRED)
if (NOT TARGET osqp)
  message(STATUS "${PROJECT_NAME} : `osqp` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
      osqp
      GIT_REPOSITORY https://github.com/oxfordcontrol/osqp.git
      GIT_TAG        origin/master
  )
  FetchContent_MakeAvailable(osqp)
  else()
    message(STATUS "${PROJECT_NAME} : `osqp` targets found.")
endif()
# setupDependencies(
#     TARGET ${PROJECT_NAME}
#     LIBS osqp gtest
#     TESTS ${BUILD_TESTS}
# )
set(FLAGS "")
setupFlags(FLAGS)

# Setup lib and exe filename
set(LIBRARY_NAME ${PROJECT_NAME})
set(EXEC_NAME ${PROJECT_NAME}_main)
list(APPEND ${LIBRARY_NAME}_SRCS
  src/${PROJECT_NAME}.cpp
)
list(APPEND ${EXEC_NAME}_SRCS
  src/${PROJECT_NAME}_main.cpp
)

# Library
add_library(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SRCS})
target_link_libraries(${LIBRARY_NAME} PUBLIC ${Eigen3_LIBRARIES} osqp)
target_include_directories(${LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


# Executable
add_executable(${EXEC_NAME} ${${EXEC_NAME}_SRCS})
target_link_libraries(${EXEC_NAME} PUBLIC ${LIBRARY_NAME})
target_include_directories(${EXEC_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Testing
if (BUILD_TESTS)
  AppendTestFlags(FLAGS)
  message(STATUS "Flags values : ${FLAGS}")
  enable_testing()
  if (NOT TARGET gtest OR NOT TARGET gmock OR NOT TARGET gtest_main)
    message(STATUS "${PROJECT_NAME} : `gtest` targets not found. Attempting to fetch contents...")
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        origin/master
    )
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)
  endif()
  # Add executable
  add_executable(${PROJECT_NAME}_tests tests/run_all_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_tests gtest gmock gtest_main)
  gtest_discover_tests(${PROJECT_NAME}_tests)
  message(STATUS "${PROJECT_NAME}: Added tests.")
endif()


