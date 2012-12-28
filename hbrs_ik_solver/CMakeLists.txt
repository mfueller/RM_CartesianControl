cmake_minimum_required(VERSION 2.4.6)
project(RM_CC)

link_directories(/usr/local/lib /usr/local/include)

include (${PROJECT_SOURCE_DIR}/config/FindPkgConfig.cmake)
include (${PROJECT_SOURCE_DIR}/config/FindEigen.cmake)
## Find KDL
set( KDL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The KDL installation directory.")
include (${PROJECT_SOURCE_DIR}/config/FindKDL.cmake)

include_directories(${PROJECT_SOURCE_DIR}/common/include)
add_library(hbrs_ik_solver common/src/hbrs_ik_solver.cpp)
target_link_libraries(hbrs_ik_solver orocos-kdl)

