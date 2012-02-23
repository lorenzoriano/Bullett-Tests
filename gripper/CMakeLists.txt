project(tutorials)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release")

include_directories("/usr/local/include/bullet")
include_directories("/usr/local/include/OpenGL")
include_directories("include/")
link_directories("/usr/local/lib")
set(libbullet BulletSoftBody BulletDynamics BulletCollision LinearMath)
set(libopengl OpenGLSupport glut GL GLU)
set(libcollada BulletColladaConverter ColladaDom XML)

add_executable(tutorials src/main.cpp)
add_executable(demo src/ccd_demo.cpp src/CcdPhysicsDemo.cpp)
add_executable(collada_demo src/ColladaDemo.cpp)

target_link_libraries(tutorials ${libbullet})
target_link_libraries(demo ${libopengl} ${libbullet})
target_link_libraries(collada_demo ${libbullet} ${libopengl} ${libcollada})

