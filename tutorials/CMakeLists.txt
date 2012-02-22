project(tutorials)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release")

include_directories("/usr/local/bullet/include/bullet")
include_directories("/usr/local/bullet/include/OpenGL")
link_directories("/usr/local/bullet/lib")
set(libraries BulletSoftBody BulletDynamics BulletCollision LinearMath)
set(opengl OpenGLSupport glut GL GLU)

add_executable(tutorials main.cpp)
add_executable(demo ccd_demo.cpp CcdPhysicsDemo.cpp)

target_link_libraries(tutorials ${libraries})
target_link_libraries(demo ${opengl} ${libraries})

