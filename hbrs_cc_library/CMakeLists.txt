cmake_minimum_required(VERSION 2.4.6)
project(HBRS_CC_LIBRARY)

link_directories(/usr/local/lib /usr/local/include)


include (${PROJECT_SOURCE_DIR}/config/FindPkgConfig.cmake)
include (${PROJECT_SOURCE_DIR}/config/FindEigen.cmake)
## Find KDL
set( KDL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The KDL installation directory.")
include (${PROJECT_SOURCE_DIR}/config/FindKDL.cmake)

include_directories(${PROJECT_SOURCE_DIR}/common/include)
include_directories(${PROJECT_SOURCE_DIR}/../hbrs_ik_solver/common/include)
add_library(cc_library /common/src/hbrs_cc_library.cpp)
#add_executable(cc_library src/cc_library.cpp)
add_library(hbrs_ik_solver ../hbrs_ik_solver/common/src/hbrs_ik_solver.cpp)
#target_link_libraries(test orocos-kdl)

