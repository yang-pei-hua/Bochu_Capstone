cmake_minimum_required(VERSION 3.24)

project(CadPointCloudClient VERSION 0.1.0 LANGUAGES CXX)

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

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets)
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
    src/app/MainWindow.h
    src/app/MainWindow.cpp
    src/app/ModelingController.h
    src/app/ModelingController.cpp
    src/app/CaptureController.h
    src/app/CaptureController.cpp
    src/app/ProjectPaths.h
    src/app/ProjectPaths.cpp
    src/app/ReconstructionController.h
    src/app/ReconstructionController.cpp
    src/core/Scene.h
    src/core/Scene.cpp
    src/core/CameraController.h
    src/core/CameraController.cpp
    src/core/OrbitCamera.h
    src/core/OrbitCamera.cpp
    src/core/SketchFrame.h
    src/core/SketchFrame.cpp
    src/io/ModelLoader.h
    src/io/PointCloudAdapter.h
    src/io/PointCloudAdapter.cpp
    src/io/PlyReader.h
    src/io/PlyReader.cpp
    src/io/CameraInfoBuilder.h
    src/io/CameraInfoBuilder.cpp
    src/rendering/BodyActor.h
    src/rendering/BodyActor.cpp
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
    src/ui/panels/ScenePanel.h
    src/ui/panels/ScenePanel.cpp
    src/ui/panels/PropertyPanel.h
    src/ui/panels/PropertyPanel.cpp
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

target_include_directories(CadPointCloudClient PRIVATE src)
target_link_libraries(CadPointCloudClient PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    ${VTK_LIBRARIES}
    CadModelCore::CadModelCore
    GeometricReconstruction::GeometricReconstruction
)

if(MSVC)
    target_compile_options(CadPointCloudClient PRIVATE /W4 /permissive- /utf-8)
endif()

# The repository-local Qt/VTK/OpenCASCADE SDKs are not on PATH and Windows
# resolves dependency DLLs from the executable directory, so stage the runtime
# DLLs next to the binary. OpenCASCADE is located here only to find its runtime
# directory - its libraries still come from CadModelCore.
if(WIN32)
    find_package(OpenCASCADE REQUIRED)

    add_custom_command(TARGET CadPointCloudClient POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:CadPointCloudClient>
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
