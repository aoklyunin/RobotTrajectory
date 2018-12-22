cmake_minimum_required(VERSION 3.5.1)
project(scene_description)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem)



include_directories(
        lumber/include
        include

        ../misc/include
        ../urdf_reader/include

        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

link_directories(
        ${Boost_LIBRARY_DIR}
        misc
        urdf_reader
)

add_library(
        ${PROJECT_NAME}
        lumber/src/scene_description.cpp
        lumber/include/scene_description.h
        src/urdf_scene_description.cpp
        include/urdf_scene_description.h
)




add_executable(testURDFSceneDescription
        test/test_urdf_scene_description.cpp
        include/urdf_scene_description.h
        src/urdf_scene_description.cpp
        lumber/src/scene_description.cpp
        lumber/include/scene_description.h)

target_link_libraries(testURDFSceneDescription
        misc
        urdf_reader
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        tbbmalloc_proxy
        tbbmalloc
        -fsanitize=address
        -lboost_filesystem
        -lboost_system
        )


add_executable(testIO ../scene_description/test/test_io.cpp)
target_link_libraries(testIO misc)

add_test(NAME testIO COMMAND testIO)
add_test(NAME testURDFSceneDescription COMMAND testURDFSceneDescription)
