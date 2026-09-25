cmake_minimum_required(VERSION 3.24)

project(CadModelCore VERSION 0.1.0 LANGUAGES CXX)

option(CAD_MODEL_CORE_BUILD_TESTS "Build the standalone modeling-core smoke test" ON)

find_package(OpenCASCADE REQUIRED COMPONENTS
    FoundationClasses
    ModelingData
    ModelingAlgorithms
    Visualization
    ApplicationFramework
    DataExchange
)

add_library(CadModelCore
    src/ModelingCore.cpp
    src/PartDocument.cpp
    src/RebuildEngine.cpp
    src/StepExporter.cpp
    src/occt/CutBuilder.cpp
    src/occt/ExtrudeBuilder.cpp
    src/occt/ShapeOps.cpp
    src/occt/SketchBuilder.cpp
)

add_library(CadModelCore::CadModelCore ALIAS CadModelCore)

target_compile_features(CadModelCore PUBLIC cxx_std_17)
target_include_directories(CadModelCore
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCASCADE_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_directories(CadModelCore PRIVATE ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(CadModelCore PUBLIC
    TKernel
    TKMath
    TKG2d
    TKG3d
    TKGeomBase
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKPrim
    TKBO
    TKShHealing
    TKBool
    TKDESTEP
)

if(MSVC)
    target_compile_options(CadModelCore PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(CadModelCore PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CAD_MODEL_CORE_BUILD_TESTS)
    include(CTest)

    add_executable(modeling_core_smoke_test tests/modeling_core_smoke_test.cpp)
    target_link_libraries(modeling_core_smoke_test PRIVATE CadModelCore::CadModelCore)
    target_include_directories(modeling_core_smoke_test PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    target_compile_definitions(modeling_core_smoke_test PRIVATE
        MODELING_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output"
    )

    if(MSVC)
        target_compile_options(modeling_core_smoke_test PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(modeling_core_smoke_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME modeling_core_smoke_test COMMAND modeling_core_smoke_test)
endif()
