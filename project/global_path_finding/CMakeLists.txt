cmake_minimum_required(VERSION 3.5.1)
project(global_path_finding)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)
set(JSONCPP_LIBRARIES jsoncpp)
cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)


#install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
        include
        include
        ../local_path_finding/include
        ../local_path_finding/lumber/include

        ../scene_description/include
        ../scene_description/lumber/include
        ../collider/lumber/include
        ../collider/include
        ../scene_wrapper/include
        ../misc/include
        ../urdf_reader/include
        ../solid3/include


        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

link_directories(
        local_path_finding
        misc
        collider
        urdf_reader
        scene_wrapper
        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/global_path_finder.cpp
)


add_executable(testTraPathFinder
        test/test_tra_path_finder.cpp
        include/global_path_finder.h
        src/global_path_finder.cpp
        )

target_link_libraries(testTraPathFinder
        local_path_finding
        misc
        scene_wrapper
        scene_description
        collider
        urdf_reader
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        pthread
        tbbmalloc_proxy
        tbbmalloc
        #-fsanitize=address
        )

add_executable(testTraPathFinder2
        test/test_tra_path_finder_2.cpp
        include/global_path_finder.h
        src/global_path_finder.cpp
        )

target_link_libraries(testTraPathFinder2
        local_path_finding
        misc
        scene_wrapper
        scene_description
        collider
        urdf_reader
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        pthread
        tbbmalloc_proxy
        tbbmalloc
        #-fsanitize=address
        )

#add_test(NAME testTraPathFinder COMMAND testTraPathFinder)
#
#add_test(NAME testTraPathFinder2 COMMAND testTraPathFinder2)
#
#




