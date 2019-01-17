cmake_minimum_required(VERSION  3.5.1)


PROJECT(FindPath CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${PROJECT_SOURCE_DIR}")




enable_testing()




add_compile_options(
        -O3
        -g3
        #-Wall
        )

add_subdirectory(project/solid3)
include_directories(solid3/include)

add_subdirectory(project/misc)
include_directories(misc/include)

add_subdirectory(project/urdf_reader)
include_directories(urdf_reader/include)

add_subdirectory(project/scene_description)
include_directories(scene_description/include)

add_subdirectory(project/scene_wrapper)
include_directories(scene_wrapper/include)

add_subdirectory(project/collider)
include_directories(collider/include)

add_subdirectory(project/local_path_finding)
include_directories(local_path_finding/include)

add_subdirectory(project/visualization)
include_directories(visualisation/include)

add_subdirectory(project/global_path_finding)
include_directories(global_path_finding/include)

add_subdirectory(project/generator)
include_directories(generator/include)


add_subdirectory(project/scene_editor)
include_directories(scene_editor/include)

add_subdirectory(project/path_finding_client)
include_directories(path_finding_client/include)


add_subdirectory(project/force_torque)
include_directories(force_torque/include)


add_subdirectory(project/graphic_template)
include_directories(graphic_template/include)