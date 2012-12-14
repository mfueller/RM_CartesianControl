cmake_minimum_required(VERSION 2.4.6)
project(HBRS_CC_LIBRARY)

link_directories(/usr/local/lib /usr/local/include)


#include (${PROJECT_SOURCE_DIR}/config/FindPkgConfig.cmake)
## Find KDL
#set( KDL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The KDL installation directory.")
#include (${PROJECT_SOURCE_DIR}/config/FindKDL.cmake)

include_directories(${PROJECT_SOURCE_DIR}/common/include)
add_library(cc_library common/src/hbrs_cc_library.cpp)
#add_executable(cc_library src/cc_library.cpp)
#target_link_libraries(test orocos-kdl)

