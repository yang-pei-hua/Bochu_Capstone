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
    src/MainWindow.h
    src/MainWindow.cpp
    src/app/DemoModelIds.h
    src/app/ModelingController.h
    src/app/ModelingController.cpp
    src/core/Scene.h
    src/core/Scene.cpp
    src/core/CameraController.h
    src/core/CameraController.cpp
    src/io/ModelLoader.h
    src/rendering/BodyActor.h
    src/rendering/BodyActor.cpp
    src/rendering/OcctShapeTessellator.h
    src/rendering/OcctShapeTessellator.cpp
    src/widgets/VTKViewer.h
    src/widgets/VTKViewer.cpp
    src/widgets/ModelingPanel.h
    src/widgets/ModelingPanel.cpp
    src/widgets/ScenePanel.h
    src/widgets/ScenePanel.cpp
    src/widgets/PropertyPanel.h
    src/widgets/PropertyPanel.cpp
    src/widgets/CameraPanel.h
    src/widgets/CameraPanel.cpp
    src/widgets/RenderPanel.h
    src/widgets/RenderPanel.cpp
)

target_include_directories(CadPointCloudClient PRIVATE src)
target_link_libraries(CadPointCloudClient PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    ${VTK_LIBRARIES}
    CadModelCore::CadModelCore
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
