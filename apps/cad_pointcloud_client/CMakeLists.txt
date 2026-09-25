cmake_minimum_required(VERSION 3.24)

project(CadPointCloudClient VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CAD_CLIENT_LOCAL_VTK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/vtk-9.7.0")
if(EXISTS "${CAD_CLIENT_LOCAL_VTK_ROOT}/lib/cmake/vtk-9.7/vtk-config.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${CAD_CLIENT_LOCAL_VTK_ROOT}")
endif()

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
    src/core/Scene.h
    src/core/Scene.cpp
    src/core/CameraController.h
    src/core/CameraController.cpp
    src/io/ModelLoader.h
    src/widgets/VTKViewer.h
    src/widgets/VTKViewer.cpp
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
)

if(MSVC)
    target_compile_options(CadPointCloudClient PRIVATE /W4 /permissive- /utf-8)
endif()

vtk_module_autoinit(
    TARGETS CadPointCloudClient
    MODULES ${VTK_LIBRARIES}
)
