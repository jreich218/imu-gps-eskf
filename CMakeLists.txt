cmake_minimum_required(VERSION 3.20)
project(state_estimation_init LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED CONFIG)

add_executable(state_estimation
  include/initialization.hpp
  include/scene_io.hpp
  include/scene_types.hpp
  include/gps_generation.hpp
  main.cpp
  src/initialization.cpp
  src/scene_io.cpp
  src/gps_generation.cpp
)

target_include_directories(state_estimation PRIVATE include)
target_link_libraries(state_estimation PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(state_estimation PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(scene_io_tests
    tests/test_scene_io.cpp
    src/scene_io.cpp
  )

  target_include_directories(scene_io_tests PRIVATE include)
  target_link_libraries(scene_io_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
    nlohmann_json::nlohmann_json
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(scene_io_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(scene_io_tests)

  add_executable(gps_generation_tests
    tests/test_gps_generation.cpp
    src/gps_generation.cpp
  )

  target_include_directories(gps_generation_tests PRIVATE include)
  target_link_libraries(gps_generation_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(gps_generation_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(gps_generation_tests)
endif()
