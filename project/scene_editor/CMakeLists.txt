cmake_minimum_required(VERSION 3.5.1)
project(scene_editor)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets CONFIG REQUIRED)


set(JSONCPP_LIBRARIES jsoncpp)


include_directories(
        include
        ../client/include
        ../speed_path_finding/include
        ../local_path_finding/lumber/include
        ../local_path_finding/include/
        ../misc/include
        ../urdf_reader/include
        ../collider/include
        ../collider/lumber/include
        ../scene_description/include
        ../scene_description/lumber/include
        ../scene_wrapper/include
        ../solid3/include
        ../scene_editor/include
        ../client/include
        ${EIGEN3_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

link_directories(
        ${Boost_LIBRARY_DIR}
        misc
        scene_wrapper
        scene_description
        local_path_finding
        collider
        urdf_reader
        pthread
        solid3
)

add_executable(pathFindingSceneEditor
        scene_editor.cpp
        ../scene_editor/src/scene_editor_widget.cpp
        ../scene_editor/include/scene_editor_widget.h
        ../scene_editor/src/scene_editor_drawer.cpp
        ../scene_editor/include/scene_editor_drawer.h
        ../scene_editor/src/scene_editor_window.cpp
        ../scene_editor/include/scene_editor_window.h
        src/scene_editor_main_window.cpp
        include/scene_editor_main_window.h
        )


target_link_libraries(pathFindingSceneEditor
        local_path_finding
        Qt5::Widgets
        scene_wrapper
        scene_description
        collider
        urdf_reader
        pthread
        solid3
        misc
        ${CMAKE_THREAD_LIBS_INIT}
        ${JSONCPP_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        -lboost_filesystem
        -lboost_system
        #-fsanitize=address
        )

