cmake_minimum_required(VERSION 3.5.1)
project(scene_wrapper)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

set(JSONCPP_LIBRARIES jsoncpp)


include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ../solid3/include
        ../misc/include
        ../urdf_reader/include
        ../collider/include
        ../collider/lumber/include
        ../scene_description/include
        ../scene_description/lumber/include
        include
)

link_directories(
        ${Boost_LIBRARY_DIR}
        misc
        urdf_reader
        collider
        scene_description
        solid3
)

add_library(
        ${PROJECT_NAME}
        src/scene_wrapper.cpp
)


