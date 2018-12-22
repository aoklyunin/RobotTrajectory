cmake_minimum_required(VERSION 3.5.1)
project(sync_path_finding)

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
        lumber/include
        ../scene_description/include
        ../scene_description/lumber/include
        ../collider/lumber/include
        ../collider/include
        ../scene_wrapper/include
        ../misc/include
        ../urdf_reader/include
        ../solid3/include
        ../local_path_finding/include
        ../local_path_finding/lumber/include
        ../global_path_finding/include
        ../global_path_finding/lumber/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
        local_path_finding
        global_path_finding
        misc
        collider
        urdf_reader
        scene_wrapper
)


add_executable(generator console_generator.cpp lumber/include/generator.h lumber/src/generator.cpp)


target_link_libraries(generator
        global_path_finding
        local_path_finding
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
        ${Boost_LIBRARIES}

        )


add_executable(aStarGenerator src/a_star_generator.cpp lumber/include/generator.h lumber/src/generator.cpp)


target_link_libraries(aStarGenerator
        global_path_finding
        local_path_finding
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
        ${Boost_LIBRARIES})




add_executable(orderedAStarGenerator src/ordered_a_star_generator.cpp lumber/include/generator.h lumber/src/generator.cpp)


target_link_libraries(orderedAStarGenerator
        global_path_finding
        local_path_finding
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
        ${Boost_LIBRARIES})




add_executable(TraGenerator src/tra_generator.cpp lumber/include/generator.h lumber/src/generator.cpp)


target_link_libraries(TraGenerator
        global_path_finding
        local_path_finding
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
        ${Boost_LIBRARIES}

        -fsanitize=address

        )

add_executable(testAStar test/testAStar.cpp lumber/include/generator.h lumber/src/generator.cpp)


target_link_libraries(testAStar
        global_path_finding
        local_path_finding
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
        ${Boost_LIBRARIES}
        # -fsanitize=address
        )

