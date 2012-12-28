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


include_directories(
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
)

include (${PROJECT_SOURCE_DIR}/config/FindPkgConfig.cmake)
include (${PROJECT_SOURCE_DIR}/config/FindEigen.cmake)
## Find KDL
set( KDL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The KDL installation directory.")
include (${PROJECT_SOURCE_DIR}/config/FindKDL.cmake)

#compile the class(es) into a library
rosbuild_add_library(my_functionality_lib common/src/my_functional_class.cpp)

#create an executable for your ros node
rosbuild_add_executable(cc_node ros/nodes/cc_node.cpp)

include_directories(${PROJECT_SOURCE_DIR}/../hbrs_cc_library/common/include)
include_directories(${PROJECT_SOURCE_DIR}/../hbrs_ik_solver/common/include)
add_library(hbrs_cc_library ../hbrs_cc_library/common/src/hbrs_cc_library.cpp)
add_library(hbrs_ik_solver ../hbrs_ik_solver/common/src/hbrs_ik_solver.cpp)
#add_library(hbrs_ik_solver common/src/hbrs_ik_solver.cpp)

#link a executable against the previously created library
target_link_libraries(cc_node my_functionality_lib hbrs_cc_library hbrs_ik_solver)


