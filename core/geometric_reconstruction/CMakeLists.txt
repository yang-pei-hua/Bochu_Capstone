cmake_minimum_required(VERSION 3.24)

project(GeometricReconstruction VERSION 0.1.0 LANGUAGES CXX)

option(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS
    "Build geometric reconstruction tests" ON)

find_package(OpenCASCADE REQUIRED)

if(NOT TARGET CadModelCore::CadModelCore)
    set(CAD_MODEL_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../parametric_modeling"
        "${CMAKE_CURRENT_BINARY_DIR}/cad_model_core"
    )
endif()

add_library(GeometricReconstruction
    src/AxisAlignedBoxRecognizer.cpp
    src/BoxRecognizer.cpp
    src/ModelingAdapter.cpp
    src/PlaneDetector.cpp
    src/PointStore.cpp
    src/SyntheticPointCloud.cpp
)
add_library(GeometricReconstruction::GeometricReconstruction ALIAS
    GeometricReconstruction)

target_compile_features(GeometricReconstruction PUBLIC cxx_std_17)
target_include_directories(GeometricReconstruction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(GeometricReconstruction PUBLIC
    CadModelCore::CadModelCore
)

if(MSVC)
    target_compile_options(GeometricReconstruction PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(GeometricReconstruction PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS)
    include(CTest)
    add_executable(geometric_reconstruction_test
        tests/geometric_reconstruction_test.cpp)
    target_link_libraries(geometric_reconstruction_test PRIVATE
        GeometricReconstruction::GeometricReconstruction)
    target_compile_definitions(geometric_reconstruction_test PRIVATE
        RECONSTRUCTION_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output"
    )
    if(MSVC)
        target_compile_options(geometric_reconstruction_test PRIVATE
            /W4 /permissive- /utf-8)
    else()
        target_compile_options(geometric_reconstruction_test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME geometric_reconstruction_test
        COMMAND geometric_reconstruction_test)
    set_tests_properties(geometric_reconstruction_test PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}"
    )
endif()
