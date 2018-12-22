cmake_minimum_required(VERSION 3.5.1)
project(local_path_finding)

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
        lumber/include
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
        scene_wrapper
        scene_description
        misc
        collider
        urdf_reader

        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        lumber/src/path_finder.cpp
        lumber/src/local_path_finder.cpp
        src/tra_star_path_finder.cpp
        src/ordered_tra_star_path_finder.cpp
        src/sync_tra_star_path_finder.cpp
)


add_executable(testAStarPathFinder
        test/test_a_star_path_finder.cpp
        include/tra_star_path_finder.h
        src/tra_star_path_finder.cpp

        lumber/src/path_finder.cpp
        lumber/include/path_finder.h
        lumber/src/local_path_finder.cpp
        lumber/include/local_path_finder.h)

target_link_libraries(testAStarPathFinder
        misc
        scene_wrapper
        scene_description
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
        -lboost_filesystem
        -lboost_system
        #-fsanitize=address
        )




add_executable(testOrderedAStarPathFinder
        lumber/src/path_finder.cpp
        lumber/include/path_finder.h
        lumber/src/local_path_finder.cpp
        lumber/include/local_path_finder.h

        test/test_ordered_a_star_path_finder.cpp

        src/tra_star_path_finder.cpp
        include/tra_star_path_finder.h
        src/ordered_tra_star_path_finder.cpp
        include/ordered_tra_star_path_finder.h
        )

target_link_libraries(testOrderedAStarPathFinder
        misc
        scene_wrapper
        scene_description
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
       #  -fsanitize=address
        )


add_executable(testSyncAStarPathFinder
        test/test_sync_a_star_path_finder.cpp
        include/sync_tra_star_path_finder.h
        src/sync_tra_star_path_finder.cpp

        include/tra_star_path_finder.h
        src/tra_star_path_finder.cpp

        lumber/src/path_finder.cpp
        lumber/src/local_path_finder.cpp
        lumber/include/local_path_finder.h
        )

target_link_libraries(testSyncAStarPathFinder
        scene_wrapper
        scene_description
        misc
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
         -fsanitize=address
        )


add_executable(testSyncCollider
        test/test_sync_collider.cpp

        )


target_link_libraries(testSyncCollider
        local_path_finding
        misc
        scene_wrapper
        scene_description
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
        #   -fsanitize=address
        )


add_executable(testFreePointFinding
        test/test_free_point_finding.cpp

        )


target_link_libraries(testFreePointFinding
        local_path_finding
        misc
        scene_wrapper
        scene_description
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
           -fsanitize=address
        )


#add_test(NAME testAStarPathFinder COMMAND testAStarPathFinder)
add_test(NAME testOrderedAStarPathFinder COMMAND testOrderedAStarPathFinder)

#add_test(NAME testSyncCollider COMMAND testSyncCollider)
#add_test(NAME testSyncAStarPathFinder COMMAND testSyncAStarPathFinder)






