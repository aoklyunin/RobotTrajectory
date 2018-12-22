cmake_minimum_required(VERSION 3.5.1)
project(collider)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
        include
        lumber/include
        ../misc/include
        ../solid3/include
        ../scene_description/lumber/include
        ${EIGEN3_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}

)

link_directories(
        misc
        solid3
)

add_library(
        ${PROJECT_NAME}
        src/solid_collider.cpp
        src/solid_sync_collider.cpp
        include/solid_sync_collider.h
        include/solid_collider.h
)

add_executable(testSolidOpenGL
        test/test_solid_openGL.cpp
        src/solid_collider.cpp
        include/solid_collider.h
        lumber/include/collider.h
        )

target_link_libraries(testSolidOpenGL
        misc
        urdf_reader
        pthread
        solid3
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        tbbmalloc_proxy
        tbbmalloc
        -fsanitize=address
        )

add_executable(testSolidCollision
        test/test_solid_collision.cpp
        src/solid_collider.cpp
        include/solid_collider.h
        lumber/include/collider.h
        )


target_link_libraries(testSolidCollision
        misc
        urdf_reader
        pthread
        solid3
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        tbbmalloc_proxy
        tbbmalloc
        #-fsanitize=address
        )

add_test(NAME testSolidCollision COMMAND testSolidCollision)
