cmake_minimum_required(VERSION 3.24)

project(GeometricReconstruction VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

option(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS
    "Build geometric reconstruction tests" ON)

find_package(OpenCASCADE REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

if(NOT TARGET CadModelCore::CadModelCore)
    set(CAD_MODEL_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../parametric_modeling"
        "${CMAKE_CURRENT_BINARY_DIR}/cad_model_core"
    )
endif()

add_library(GeometricReconstruction
    src/analysis/AnalysisSnapshot.cpp
    src/analysis/RepairSnapshot.cpp
    src/recognition/RepairTransaction.cpp
    src/analysis/LocalRecovery.cpp
    src/analysis/RelationOptimization.cpp
    src/analysis/SurfaceAnalysis.cpp
    src/analysis/SurfaceGeometry.cpp
    src/analysis/SurfacePatches.cpp
    src/analysis/SurfaceRelations.cpp
    src/data/PointStore.cpp
    src/detection/PrimitiveDetector.cpp
    src/detection/MultiScaleDetector.cpp
    src/detection/cgal/CgalEfficientRansacDetector.cpp
    src/detection/cgal/CurvedEvidence.cpp
    src/detection/cgal/CylinderEvidence.cpp
    src/detection/cgal/EvidenceGeometry.cpp
    src/detection/cgal/PlaneEvidence.cpp
    src/detection/cgal/RansacOptions.cpp
    src/modeling/ModelingAdapter.cpp
    src/preprocessing/Downsampling.cpp
    src/preprocessing/NormalEstimation.cpp
    src/preprocessing/OutlierFilter.cpp
    src/preprocessing/PointCloudPreprocessor.cpp
    src/recognition/AxisAlignedBoxRecognizer.cpp
    src/recognition/BoxRecognizer.cpp
    src/recognition/ParametricSolidRecognizer.cpp
    src/recognition/assembly/AssemblyReconstruction.cpp
    src/recognition/assembly/FiniteBoundaries.cpp
    src/recognition/assembly/PartClosure.cpp
    src/recognition/ThroughHoleRecognizer.cpp
    src/recognition/compound/BoundaryValidation.cpp
    src/recognition/compound/BoxCover.cpp
    src/recognition/compound/CompoundRecognizer.cpp
    src/recognition/compound/MaterialCells.cpp
    src/recognition/compound/PlanarFrame.cpp
    src/recognition/compound/PlanarGeometry.cpp
    src/recognition/curved/BoundaryAssessment.cpp
    src/recognition/curved/CapDetection.cpp
    src/recognition/curved/CurvedFitting.cpp
    src/recognition/curved/CurvedSolidRecognizer.cpp
    src/recognition/guided/GuidedSolidRecognizer.cpp
    src/recognition/guided/HintFitting.cpp
    src/recognition/guided/HintValidation.cpp
    src/recognition/mixed/AttachedFeatures.cpp
    src/recognition/mixed/HostDetection.cpp
    src/recognition/mixed/MixedFeatureRecognizer.cpp
    src/recognition/mixed/MixedMaterial.cpp
    src/testing/SyntheticPointCloud.cpp
)
add_library(GeometricReconstruction::GeometricReconstruction ALIAS
    GeometricReconstruction)

target_compile_features(GeometricReconstruction PUBLIC cxx_std_17)
target_include_directories(GeometricReconstruction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(GeometricReconstruction PUBLIC
    CadModelCore::CadModelCore
    PRIVATE
    CGAL::CGAL
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(GeometricReconstruction PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(GeometricReconstruction PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS)
    include(CTest)
    add_executable(assembly_reconstruction_test tests/assembly_reconstruction_test.cpp)
    target_link_libraries(assembly_reconstruction_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME assembly_reconstruction_test COMMAND assembly_reconstruction_test)
    set_tests_properties(assembly_reconstruction_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    add_executable(surface_analysis_test tests/analysis/surface_analysis_test.cpp)
    target_include_directories(surface_analysis_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(surface_analysis_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME surface_analysis_test COMMAND surface_analysis_test)
    set_tests_properties(surface_analysis_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    include(cmake/RegressionSources.cmake)
    add_executable(geometric_reconstruction_test ${GEOMETRIC_RECONSTRUCTION_REGRESSION_SOURCES})
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
