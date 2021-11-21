project(autonomous_guidance)

cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 20)

# Options
option(BUILD_TESTS "Want to build tests?" ON)

# Dependency management
include(cmake/setup.cmake)
include(FetchContent)

# Setup dependencies and flags (aka you have to install them!)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED) # is the only that fill ${*_LIBRARIES}
find_package(jsoncpp REQUIRED)
find_package(glog REQUIRED)

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

set(FLAGS "")
setupFlags(FLAGS)

# Setup lib and exe filename
set(LIBRARY_NAME ${PROJECT_NAME})
set(EXEC_NAME main)
list(APPEND ${LIBRARY_NAME}_SRCS
  src/utils/utils.cpp
  src/model/vehicle.cpp
  src/control/pid.cpp
  src/control/pure_pursuit.cpp
  # src/control/stanley_steer.cpp
  # src/control/nmpc_ipopt_wrapper.cpp
  # src/control/mpc_osqp_wrapper.cpp
  src/simulator/simulator.cpp
)
list(APPEND ${EXEC_NAME}_SRCS
  src/main.cpp
)

# Library
add_library(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SRCS})
target_link_libraries(${LIBRARY_NAME} PUBLIC
  Eigen3::Eigen
  osqp
  jsoncpp
  glog
)
target_include_directories(${LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


# Executable
add_executable(${EXEC_NAME} ${${EXEC_NAME}_SRCS})
target_link_libraries(${EXEC_NAME} PUBLIC ${LIBRARY_NAME} ${PYTHON_LIBRARIES})
target_include_directories(${EXEC_NAME} PUBLIC ${PYTHON_INCLUDE_DIRS} ${glog_LIBRARIES})
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


