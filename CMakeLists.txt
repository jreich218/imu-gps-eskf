cmake_minimum_required(VERSION 3.20)
project(imu-gps-eskf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED CONFIG)

add_executable(imu-gps-eskf
  include/initialization.hpp
  include/scene_io.hpp
  include/scene_types.hpp
  include/gps_generation.hpp
  include/eskf.hpp
  include/runtime_loop.hpp
  include/outputs.hpp
  main.cpp
  src/initialization.cpp
  src/scene_io.cpp
  src/gps_generation.cpp
  src/eskf.cpp
  src/runtime_loop.cpp
  src/outputs.cpp
)

target_include_directories(imu-gps-eskf PRIVATE include)
target_link_libraries(imu-gps-eskf PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(imu-gps-eskf PRIVATE -Wall -Wextra -Wpedantic)
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

  add_executable(initialization_tests
    tests/test_initialization.cpp
    src/initialization.cpp
  )

  target_include_directories(initialization_tests PRIVATE include)
  target_link_libraries(initialization_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(initialization_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(initialization_tests)

  add_executable(eskf_tests
    tests/test_eskf.cpp
    src/eskf.cpp
  )

  target_include_directories(eskf_tests PRIVATE include)
  target_link_libraries(eskf_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(eskf_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(eskf_tests)

  add_executable(runtime_loop_tests
    tests/test_runtime_loop.cpp
    src/runtime_loop.cpp
    src/eskf.cpp
  )

  target_include_directories(runtime_loop_tests PRIVATE include)
  target_link_libraries(runtime_loop_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(runtime_loop_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(runtime_loop_tests)

  add_executable(outputs_tests
    tests/test_outputs.cpp
    src/outputs.cpp
  )

  target_include_directories(outputs_tests PRIVATE include)
  target_link_libraries(outputs_tests PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
    nlohmann_json::nlohmann_json
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(outputs_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(outputs_tests)

  add_executable(app_integration_tests
    tests/test_app_integration.cpp
  )

  add_dependencies(app_integration_tests imu-gps-eskf)

  target_link_libraries(app_integration_tests PRIVATE
    GTest::gtest_main
    nlohmann_json::nlohmann_json
  )

  target_compile_definitions(app_integration_tests PRIVATE
    IMU_GPS_ESKF_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    IMU_GPS_ESKF_EXE_PATH="$<TARGET_FILE:imu-gps-eskf>"
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(app_integration_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(app_integration_tests)

endif()
