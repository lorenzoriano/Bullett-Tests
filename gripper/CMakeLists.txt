project(tutorials)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Debug")

include_directories("/usr/local/include/bullet")
include_directories("/usr/local/include/OpenGL")
include_directories("/usr/local/include/collada-dom")
include_directories("include/")

link_directories("/usr/local/lib")
set(libbullet BulletWorldImporter BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath)
set(libopengl OpenGLSupport glut GL GLU)
set(libcollada ColladaDom xml2)

add_executable(tutorials src/main.cpp)
add_executable(test_csv src/test_csv.cpp)
add_executable(test_yaml src/test_yaml.cpp)
add_executable(demo src/ccd_demo.cpp src/CcdPhysicsDemo.cpp)
add_executable(load_gripper src/loadgripper.cpp src/ColladaConverter.cpp)

target_link_libraries(tutorials ${libbullet})
target_link_libraries(demo ${libopengl} ${libbullet})
target_link_libraries(load_gripper ${libcollada} ${libopengl} ${libbullet})
target_link_libraries(test_yaml yaml-cpp ${libbullet}) 
