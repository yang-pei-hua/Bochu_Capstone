cmake_minimum_required(VERSION 3.24)

project(CadPointCloudClient VERSION 0.2.2 LANGUAGES CXX)

option(CAD_CLIENT_BUILD_TESTS "Build ParamCAD Studio I/O tests" OFF)
option(CAD_CLIENT_BUILD_VALIDATION_TOOLS "Build standard STEP and CAD validation tools" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CAD_CLIENT_LOCAL_VTK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/vtk-9.7.0")
if(EXISTS "${CAD_CLIENT_LOCAL_VTK_ROOT}/lib/cmake/vtk-9.7/vtk-config.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${CAD_CLIENT_LOCAL_VTK_ROOT}")
endif()

# The repository-local OpenCASCADE SDK lives next to the VTK one. Only set the
# default when the caller did not point CMake at another OCCT build.
set(CAD_CLIENT_LOCAL_OCCT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/occt-8.0.1")
if(NOT OpenCASCADE_DIR AND EXISTS "${CAD_CLIENT_LOCAL_OCCT_ROOT}/cmake/OpenCASCADEConfig.cmake")
    set(OpenCASCADE_DIR "${CAD_CLIENT_LOCAL_OCCT_ROOT}/cmake" CACHE PATH
        "Directory that contains OpenCASCADEConfig.cmake")
endif()

# The parametric modeling core is consumed as a normal CMake subproject; the
# client never re-lists the OpenCASCADE libraries it needs.
set(CAD_MODEL_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../core/parametric_modeling"
    "${CMAKE_CURRENT_BINARY_DIR}/cad_model_core"
)

# Point-cloud reconstruction is the bridge from observed points to the modeling
# core. Its own test target is not built here: the client consumes the library.
set(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../core/geometric_reconstruction"
    "${CMAKE_CURRENT_BINARY_DIR}/geometric_reconstruction"
)
if(CAD_CLIENT_BUILD_TESTS)
    enable_testing()
    add_executable(surface_analysis_test ../../core/geometric_reconstruction/tests/analysis/surface_analysis_test.cpp)
    target_link_libraries(surface_analysis_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    target_include_directories(surface_analysis_test PRIVATE ${OpenCASCADE_INCLUDE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/../../core/geometric_reconstruction/src")
    target_link_directories(surface_analysis_test PRIVATE ${OpenCASCADE_LIBRARY_DIR})
    add_test(NAME surface_analysis_test COMMAND surface_analysis_test)
    include("${CMAKE_CURRENT_SOURCE_DIR}/../../core/geometric_reconstruction/cmake/RegressionSources.cmake")
    add_executable(geometric_reconstruction_regression_test ${GEOMETRIC_RECONSTRUCTION_REGRESSION_SOURCES})
    target_link_libraries(geometric_reconstruction_regression_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    target_compile_definitions(geometric_reconstruction_regression_test PRIVATE
        RECONSTRUCTION_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/shared-surface-regression")
    add_test(NAME geometric_reconstruction_regression_test COMMAND geometric_reconstruction_regression_test)
    set_tests_properties(surface_analysis_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    set_tests_properties(geometric_reconstruction_regression_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
endif()

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets)
# OpenCASCADE is resolved here as well as inside CadModelCore because the client
# names the STEP data-exchange toolkits its importer uses directly.
find_package(OpenCASCADE REQUIRED)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersSources
    GUISupportQt
    ImagingCore
    InteractionStyle
    InteractionWidgets
    IOImage
    RenderingAnnotation
    RenderingCore
    RenderingOpenGL2
)

qt_add_executable(CadPointCloudClient WIN32
    src/main.cpp
    resources/app.qrc
    src/app/MainWindow.h
    src/app/MainWindow.cpp
    src/app/WorkspaceController.h
    src/app/CadReconstructionTask.h
    src/app/CadReconstructionTask.cpp
    src/app/CapturePlan.h
    src/app/SketchWorkspaceController.h
    src/app/SketchWorkspaceController.cpp
    src/app/PointCloudWorkspaceController.h
    src/app/PointCloudWorkspaceController.cpp
    src/app/CaptureWorkspaceController.h
    src/app/CaptureWorkspaceController.cpp
    src/app/ModelViewController.h
    src/app/ModelViewController.cpp
    src/app/ModelingController.h
    src/app/ModelingController.cpp
    src/app/FeatureEditingController.h
    src/app/FeatureEditingController.cpp
    src/app/CaptureController.h
    src/app/CaptureController.cpp
    src/app/ProjectController.h
    src/app/ProjectController.cpp
    src/app/ProjectPaths.h
    src/app/ProjectPaths.cpp
    src/app/ReconstructionController.h
    src/app/ReconstructionController.cpp
    src/core/Scene.h
    src/core/Scene.cpp
    src/core/PointCloudDocument.h
    src/core/PointCloudDocument.cpp
    src/core/PointCloudSelection.h
    src/core/PointCloudSelection.cpp
    src/core/PointCloudEdits.cpp src/io/PlyWriter.cpp
    src/core/PointCloudRelations.cpp
    src/core/CameraController.h
    src/core/CameraController.cpp
    src/core/OrbitCamera.h
    src/core/OrbitCamera.cpp
    src/core/SketchFrame.h
    src/core/SketchFrame.cpp
    src/core/TextParameterCommand.h src/core/TextParameterCommand.cpp
    src/io/FeatureCodec.h
    src/io/FeatureCodec.cpp
    src/io/ProjectStore.h
    src/io/ProjectStore.cpp
    src/core/PresentationState.h
    src/io/ModelLoader.h
    src/io/PointCloudAdapter.h
    src/io/PointCloudAdapter.cpp
    src/io/PointCloudMetadata.h
    src/io/PointCloudAnalysis.h
    src/io/PointCloudAnalysis.cpp
    src/io/StepModelLoader.h
    src/io/StepModelLoader.cpp
    src/io/StepModelExporter.h
    src/io/StepModelExporter.cpp
    src/io/PlyReader.h
    src/io/PlyReader.cpp
    src/io/CameraInfoBuilder.h
    src/io/CameraInfoBuilder.cpp
    src/rendering/BodyActor.h
    src/rendering/BodyActor.cpp
    src/rendering/BoxReferenceActor.h
    src/rendering/BoxReferenceActor.cpp
    src/rendering/DatumPlaneActor.h
    src/rendering/DatumPlaneActor.cpp
    src/rendering/PointCloudActor.h
    src/rendering/PointCloudActor.cpp
    src/rendering/SketchActor.h
    src/rendering/SketchActor.cpp
    src/rendering/OcctShapeTessellator.h
    src/rendering/OcctShapeTessellator.cpp
    src/ui/viewport/ViewportTool.h
    src/ui/viewport/SketchInteractorStyle.h
    src/ui/viewport/SketchInteractorStyle.cpp
    src/ui/viewport/VTKViewer.h
    src/ui/viewport/VTKViewer.cpp
    src/ui/panels/ModelingPanel.h
    src/ui/panels/ModelingPanel.cpp
    src/ui/panels/FeatureDetailsPanel.h
    src/ui/panels/FeatureDetailsPanel.cpp
    src/ui/panels/FeatureEditorPanel.h
    src/ui/panels/FeatureEditorPanel.cpp
    src/ui/panels/TextParameterPanel.h src/ui/panels/TextParameterPanel.cpp
    src/ui/panels/ScenePanel.h
    src/ui/panels/ScenePanel.cpp
    src/ui/panels/PropertyPanel.h
    src/ui/panels/PropertyPanel.cpp
    src/ui/panels/PointCloudPanel.h
    src/ui/panels/PointCloudPanel.cpp
    src/ui/panels/PointCloudRelationsPanel.h
    src/ui/panels/PointCloudRelationsPanel.cpp
    src/ui/panels/CameraPanel.h
    src/ui/panels/CameraPanel.cpp
    src/ui/panels/CapturePanel.h
    src/ui/panels/CapturePanel.cpp
    src/ui/panels/ReconstructPanel.h
    src/ui/panels/ReconstructPanel.cpp
    src/ui/panels/RenderPanel.h
    src/ui/panels/RenderPanel.cpp
    src/ui/panels/SketchPanel.h
    src/ui/panels/SketchPanel.cpp
    src/ui/panels/SketchEntityPanel.h
    src/ui/panels/SketchEntityPanel.cpp
    src/ui/panels/SketchToolPalette.h
    src/ui/panels/SketchToolPalette.cpp
)

set_target_properties(CadPointCloudClient PROPERTIES
    OUTPUT_NAME "ParamCAD Studio"
)

if(WIN32)
    configure_file(resources/app-icon.rc.in
        "${CMAKE_CURRENT_BINARY_DIR}/app-icon.rc" @ONLY)
    target_sources(CadPointCloudClient PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/app-icon.rc")
    set_property(SOURCE "${CMAKE_CURRENT_BINARY_DIR}/app-icon.rc" APPEND PROPERTY
        OBJECT_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/paramcad-studio.ico")
endif()

target_include_directories(CadPointCloudClient PRIVATE src)
# The STEP importer names OCCT toolkits directly; CadModelCore already
# contributes TKernel/TKBRep/TKTopAlgo and the OCCT include directories, so only
# the data-exchange and meshing toolkits are added on top of it.
target_link_directories(CadPointCloudClient PRIVATE ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(CadPointCloudClient PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    ${VTK_LIBRARIES}
    CadModelCore::CadModelCore
    GeometricReconstruction::GeometricReconstruction
    TKDE
    TKXSBase
    TKDESTEP
    TKMesh
)

if(MSVC)
    target_compile_options(CadPointCloudClient PRIVATE /W4 /permissive- /utf-8)
endif()

# The repository-local Qt/VTK/OpenCASCADE SDKs are not on PATH and Windows
# resolves dependency DLLs from the executable directory, so stage the runtime
# DLLs next to the binary.
if(WIN32)
    # CGAL is linked through a static library, so CMake's transitive runtime
    # scan does not consistently discover GMP. Locate the vcpkg runtime
    # explicitly so a normal developer build is launchable without a manual
    # DLL copy (the packaging script performs the same validation).
    find_file(CAD_CLIENT_GMP_RUNTIME
        NAMES gmp-10.dll
        PATHS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin"
        NO_DEFAULT_PATH
    )
    if(NOT CAD_CLIENT_GMP_RUNTIME)
        message(FATAL_ERROR
            "gmp-10.dll was not found in the active vcpkg x64-windows installation")
    endif()

    add_custom_command(TARGET CadPointCloudClient POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:CadPointCloudClient>
                "$<TARGET_FILE_DIR:CadPointCloudClient>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CAD_CLIENT_GMP_RUNTIME}"
                "$<TARGET_FILE_DIR:CadPointCloudClient>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${OpenCASCADE_BINARY_DIR}"
                "$<TARGET_FILE_DIR:CadPointCloudClient>"
        # Qt platform and style plugins live outside the Qt bin directory.
        COMMAND $<TARGET_FILE:Qt6::windeployqt>
                --no-translations
                "$<TARGET_FILE:CadPointCloudClient>"
        COMMAND_EXPAND_LISTS
        COMMENT "Staging Qt, VTK and OpenCASCADE runtime DLLs"
        VERBATIM
    )
endif()

vtk_module_autoinit(
    TARGETS CadPointCloudClient
    MODULES ${VTK_LIBRARIES}
)

if(CAD_CLIENT_BUILD_VALIDATION_TOOLS)
    add_executable(cad_reconstruction_validation
        ../../tools/cad_reconstruction_validation.cpp
        src/io/PlyReader.cpp
        src/io/PointCloudAdapter.cpp
        src/io/StepModelLoader.cpp
    )
    target_include_directories(cad_reconstruction_validation PRIVATE src)
    target_link_directories(cad_reconstruction_validation PRIVATE ${OpenCASCADE_LIBRARY_DIR})
    target_link_libraries(cad_reconstruction_validation PRIVATE
        Qt6::Core GeometricReconstruction::GeometricReconstruction TKDE TKXSBase TKDESTEP)
    if(MSVC)
        target_compile_options(cad_reconstruction_validation PRIVATE /W4 /permissive- /utf-8)
        # Native failures get a bounded symbolized JSON stack beside the report.
        target_compile_options(cad_reconstruction_validation PRIVATE /Zi)
        target_compile_options(GeometricReconstruction PRIVATE /Zi)
        target_link_options(cad_reconstruction_validation PRIVATE /DEBUG)
        target_link_libraries(cad_reconstruction_validation PRIVATE dbghelp)
    endif()
endif()

if(CAD_CLIENT_BUILD_TESTS)
    include(CTest)
    add_executable(text_parameter_command_test tests/text_parameter_command_test.cpp src/core/TextParameterCommand.cpp)
    target_include_directories(text_parameter_command_test PRIVATE src)
    target_link_libraries(text_parameter_command_test PRIVATE Qt6::Core CadModelCore::CadModelCore)
    if(MSVC)
        target_compile_options(text_parameter_command_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME text_parameter_command_test COMMAND text_parameter_command_test)
    set_tests_properties(text_parameter_command_test PROPERTIES TIMEOUT 60
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(feature_details_test tests/feature_details_test.cpp
        src/ui/panels/TextParameterPanel.h src/ui/panels/TextParameterPanel.cpp
        src/ui/panels/FeatureEditorPanel.h src/ui/panels/FeatureEditorPanel.cpp
        src/ui/panels/FeatureDetailsPanel.h src/ui/panels/FeatureDetailsPanel.cpp
        src/ui/panels/ModelingPanel.h src/ui/panels/ModelingPanel.cpp
        src/core/SketchFrame.cpp)
    target_include_directories(feature_details_test PRIVATE src)
    target_link_libraries(feature_details_test PRIVATE Qt6::Widgets CadModelCore::CadModelCore)
    if(MSVC)
        target_compile_options(feature_details_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME feature_details_test COMMAND feature_details_test)
    set_tests_properties(feature_details_test PROPERTIES TIMEOUT 60
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(project_store_test tests/project_store_test.cpp
        src/io/ProjectStore.cpp src/io/FeatureCodec.cpp src/io/PlyWriter.cpp
        src/io/PlyReader.cpp src/io/PointCloudAdapter.cpp
        src/core/PointCloudDocument.h src/core/PointCloudDocument.cpp
        src/core/PointCloudSelection.cpp src/core/PointCloudEdits.cpp src/core/PointCloudRelations.cpp)
    target_include_directories(project_store_test PRIVATE src)
    target_link_libraries(project_store_test PRIVATE Qt6::Core Qt6::Gui
        VTK::CommonCore VTK::CommonDataModel CadModelCore::CadModelCore
        GeometricReconstruction::GeometricReconstruction)
    if(MSVC)
        target_compile_options(project_store_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME project_store_test COMMAND project_store_test)
    set_tests_properties(project_store_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(step_model_exporter_test tests/step_model_exporter_test.cpp
        src/io/StepModelExporter.cpp src/io/StepModelLoader.cpp)
    target_include_directories(step_model_exporter_test PRIVATE src)
    target_link_directories(step_model_exporter_test PRIVATE ${OpenCASCADE_LIBRARY_DIR})
    target_link_libraries(step_model_exporter_test PRIVATE
        Qt6::Core CadModelCore::CadModelCore TKDE TKXSBase TKDESTEP)
    if(MSVC)
        target_compile_options(step_model_exporter_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME step_model_exporter_test COMMAND step_model_exporter_test)
    set_tests_properties(step_model_exporter_test PROPERTIES TIMEOUT 60
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(point_cloud_document_test tests/point_cloud_document_test.cpp
        src/core/PointCloudDocument.h src/core/PointCloudDocument.cpp
        src/core/PointCloudSelection.cpp src/core/PointCloudEdits.cpp src/io/PlyWriter.cpp
        src/core/PointCloudRelations.cpp src/io/PlyReader.cpp)
    target_include_directories(point_cloud_document_test PRIVATE src)
    target_link_libraries(point_cloud_document_test PRIVATE
        Qt6::Core VTK::CommonCore VTK::CommonDataModel GeometricReconstruction::GeometricReconstruction)
    if(MSVC)
        target_compile_options(point_cloud_document_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME point_cloud_document_test COMMAND point_cloud_document_test)
    set_tests_properties(point_cloud_document_test PROPERTIES TIMEOUT 60
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(cad_reconstruction_task_test tests/cad_reconstruction_task_test.cpp
        src/app/CadReconstructionTask.h src/app/CadReconstructionTask.cpp
        src/io/PointCloudAdapter.cpp src/io/PlyReader.cpp)
    target_include_directories(cad_reconstruction_task_test PRIVATE src)
    target_link_libraries(cad_reconstruction_task_test PRIVATE
        Qt6::Core GeometricReconstruction::GeometricReconstruction)
    if(MSVC)
        target_compile_options(cad_reconstruction_task_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME cad_reconstruction_task_test COMMAND cad_reconstruction_task_test)
    set_tests_properties(cad_reconstruction_task_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(point_cloud_relations_test tests/point_cloud_relations_test.cpp
        src/core/PointCloudSelection.cpp src/core/PointCloudEdits.cpp src/io/PlyWriter.cpp src/core/PointCloudRelations.cpp src/io/PlyReader.cpp)
    target_include_directories(point_cloud_relations_test PRIVATE src)
    target_link_libraries(point_cloud_relations_test PRIVATE Qt6::Core VTK::CommonCore VTK::CommonDataModel GeometricReconstruction::GeometricReconstruction)
    add_test(NAME point_cloud_relations_test COMMAND point_cloud_relations_test)
    add_executable(point_cloud_analysis_test tests/point_cloud_analysis_test.cpp
        src/io/PointCloudAnalysis.cpp src/io/PointCloudAdapter.cpp src/io/PlyReader.cpp)
    target_include_directories(point_cloud_analysis_test PRIVATE src)
    target_link_libraries(point_cloud_analysis_test PRIVATE
        Qt6::Core VTK::CommonCore VTK::CommonDataModel GeometricReconstruction::GeometricReconstruction)
    add_test(NAME point_cloud_analysis_test COMMAND point_cloud_analysis_test)
    if(MSVC)
        target_compile_options(point_cloud_relations_test PRIVATE /W4 /permissive- /utf-8)
        target_compile_options(point_cloud_analysis_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    set_tests_properties(point_cloud_relations_test point_cloud_analysis_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    add_executable(point_cloud_selection_test
        tests/point_cloud_selection_test.cpp
        src/core/PointCloudSelection.cpp
        src/core/PointCloudEdits.cpp src/io/PlyWriter.cpp
        src/core/PointCloudRelations.cpp
    )
    target_include_directories(point_cloud_selection_test PRIVATE src)
    target_link_libraries(point_cloud_selection_test PRIVATE Qt6::Core VTK::CommonCore VTK::CommonDataModel GeometricReconstruction::GeometricReconstruction)
    if(MSVC)
        target_compile_options(point_cloud_selection_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME point_cloud_selection_test COMMAND point_cloud_selection_test)
    set_tests_properties(point_cloud_selection_test PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>"
    )
    # Exercise the actual window connections and VTK picking, not a second
    # wiring of the same widgets maintained only for tests.
    get_target_property(CAD_CLIENT_TEST_SOURCES CadPointCloudClient SOURCES)
    list(REMOVE_ITEM CAD_CLIENT_TEST_SOURCES src/main.cpp)
    foreach(interaction_test IN ITEMS point_cloud_interaction_test workspace_interaction_test project_interaction_test scene_interaction_test parameter_editing_test semantic_face_reference_test text_parameter_editing_test navigation_import_test camera_interaction_test)
        add_executable(${interaction_test}
            tests/${interaction_test}.cpp ${CAD_CLIENT_TEST_SOURCES})
        target_include_directories(${interaction_test} PRIVATE src)
        target_link_directories(${interaction_test} PRIVATE ${OpenCASCADE_LIBRARY_DIR})
        target_link_libraries(${interaction_test} PRIVATE
            Qt6::Core Qt6::Gui Qt6::Widgets ${VTK_LIBRARIES}
            CadModelCore::CadModelCore GeometricReconstruction::GeometricReconstruction
            TKDE TKXSBase TKDESTEP TKMesh)
        vtk_module_autoinit(TARGETS ${interaction_test} MODULES ${VTK_LIBRARIES})
        if(MSVC)
            target_compile_options(${interaction_test} PRIVATE /W4 /permissive- /utf-8)
        endif()
        add_test(NAME ${interaction_test} COMMAND ${interaction_test})
        set_tests_properties(${interaction_test} PROPERTIES TIMEOUT 60
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:CadPointCloudClient>")
    endforeach()
    set_tests_properties(workspace_interaction_test PROPERTIES TIMEOUT 120)
    add_executable(orbit_camera_test
        tests/orbit_camera_test.cpp
        src/core/OrbitCamera.cpp
    )
    target_include_directories(orbit_camera_test PRIVATE src)
    if(MSVC)
        target_compile_options(orbit_camera_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME orbit_camera_test COMMAND orbit_camera_test)

    add_executable(camera_info_builder_test
        tests/camera_info_builder_test.cpp
        src/core/OrbitCamera.cpp
        src/io/CameraInfoBuilder.cpp
    )
    target_include_directories(camera_info_builder_test PRIVATE src)
    target_link_libraries(camera_info_builder_test PRIVATE Qt6::Core)
    if(MSVC)
        target_compile_options(camera_info_builder_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME camera_info_builder_test COMMAND camera_info_builder_test)

    add_executable(point_cloud_io_test
        tests/point_cloud_io_test.cpp
        src/io/PlyReader.cpp
        src/io/PointCloudAdapter.cpp
    )
    target_include_directories(point_cloud_io_test PRIVATE src)
    target_link_libraries(point_cloud_io_test PRIVATE
        Qt6::Core
        GeometricReconstruction::GeometricReconstruction
    )
    if(MSVC)
        target_compile_options(point_cloud_io_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME point_cloud_io_test COMMAND point_cloud_io_test)

    # The STEP importer is a client-side reader, so it is tested where it lives.
    # OpenCASCADE also writes the sample it reads back, which keeps the repository
    # free of committed model binaries.
    add_executable(step_model_loader_test
        tests/step_model_loader_test.cpp
        src/io/StepModelLoader.cpp
    )
    target_include_directories(step_model_loader_test PRIVATE src)
    target_link_directories(step_model_loader_test PRIVATE ${OpenCASCADE_LIBRARY_DIR})
    target_link_libraries(step_model_loader_test PRIVATE
        Qt6::Core
        CadModelCore::CadModelCore
        TKDE
        TKXSBase
        TKDESTEP
    )
    if(MSVC)
        target_compile_options(step_model_loader_test PRIVATE /W4 /permissive- /utf-8)
    endif()
    add_test(NAME step_model_loader_test COMMAND step_model_loader_test)
    set_tests_properties(step_model_loader_test PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}"
    )
endif()
