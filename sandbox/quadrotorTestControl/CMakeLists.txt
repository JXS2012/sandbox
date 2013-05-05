cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(vector_computation src/vector_computation)

#rosbuild_add_library(quadrotor src/quadrotor.cpp)
#target_link_libraries(quadrotor vector_computation)

rosbuild_add_library(dummybird src/dummybird.cpp)
target_link_libraries(dummybird vector_computation)

rosbuild_add_library(birdeye src/birdeye.cpp)
target_link_libraries(birdeye vector_computation)

rosbuild_add_library(highbird src/highbird.cpp)
target_link_libraries(highbird dummybird)
target_link_libraries(highbird birdeye)

#rosbuild_add_library(smartbird src/smartbird.cpp)
#target_link_libraries(smartbird dummybird)

rosbuild_add_executable(quadSmart src/quadSmart.cpp)
target_link_libraries(quadSmart highbird)

#rosbuild_add_executable(quadFlight src/quadFlight.cpp)
#target_link_libraries(quadFlight quadrotor)

#rosbuild_add_executable(testControl src/testControl.cpp)
#rosbuild_add_executable(tfListen src/tfListen.cpp)
#rosbuild_add_executable(angleTransform src/angleTransform.cpp)
#rosbuild_add_executable(quadShift src/quadShift.cpp)
#rosbuild_add_executable(quadController src/quadController.cpp)

