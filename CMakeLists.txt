#------------------------------------------------------------
#
#  Root CMake file for ebv_dff
#  A project to perform Event Based Depth from Focus
#                ------------------------
#  From the off-tree build directory, invoke:
#  $ cmake <PATH_TO_PROJECT_ROOT>
#
#  initial version: jmartel@ini.ethz.ch @ 2017
#
#------------------------------------------------------------

# ENSURE MINIMUM VERSION TO USE------------------------------
# In this version of the CMake architecture of the EBV_PROJECT
# the directives qt5_use_modules() are used, it requires CMake
# version 2.8.9 at least.
cmake_minimum_required(VERSION 2.8.9)
#------------------------------------------------------------

# ALLOWS MORE READABLE "IF THEN ELSE" CONSTRUCTS ------------
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
#------------------------------------------------------------

# NAME OF THE ROOT SOLUTION----------------------------------
project(semester_project)
#------------------------------------------------------------

# BUILD TYPE-------------------------------------------------
set(CMAKE_BUILD_TYPE Debug)
#------------------------------------------------------------

# ADDITIONAL MODULES WE CREATED------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/CMakeModules")
#------------------------------------------------------------

# REQUIRED MODULES AND LIBRARIES-----------------------------
# LibUSB
find_package(LibUSB REQUIRED)
if(LIBUSB_FOUND)
        include_directories(${LIBUSB_INCLUDE_DIRS})
        list(APPEND 3RD_PARTY_LIBS ${LIBUSB_LIBRARIES})
endif()

# Threads
find_package(Threads REQUIRED)
list(APPEND 3RD_PARTY_LIBS ${CMAKE_THREAD_LIBS_INIT})

# Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    ExternalProject_Add(
        Eigen
        PREFIX "eigen"
        URL "http://bitbucket.org/eigen/eigen/get/3.3.3.tar.bz2"
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        BUILD_COMMAND make -j8
        )
    set(EIGEN_FOUND TRUE CACHE BOOL "Whether Eigen was found.")
    set(EIGEN_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/eigen/include/eigen3" CACHE PATH "Eigen include dir")
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    ExternalProject_Add(
        OpenCV
        PREFIX "opencv"
        GIT_REPOSITORY "https://github.com/opencv/opencv.git"
        GIT_TAG "3.3.0"
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                   -DCMAKE_BUILD_TYPE="${OPENCV_BUILD_TYPE}"
                   -DBUILD_DOCS=OFF
                   -DBUILD_PERF_TEST=OFF
                   -DBUILD_TESTS=OFF
                   -DBUILD_WITH_DEBUG_INFO=ON
        BUILD_COMMAND make -j8
        )
    set(OpenCV_FOUND TRUE CACHE BOOL "Whether OpenCV was found")
    set(OpenCV_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/opencv/include" CACHE PATH "OpenCV include dir")
    set(OpenCV_LIBS "${CMAKE_BINARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}opencv${CMAKE_STATIC_LIBRARY_SUFFIX}" CACHE FILEPATH "OpenCV lib path")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND 3RD_PARTY_LIBS ${OpenCV_LIBS})

#------------------------------------------------------------

# DOWNLOAD EXTERNAL MODULES AND LIBRARIES--------------------
include(ExternalProject)

# Libcaer
ExternalProject_Add(
    libcaer
    PREFIX "libcaer"
    GIT_REPOSITORY "https://github.com/inilabs/libcaer"
    GIT_TAG 2.5.0
    UPDATE_COMMAND ""
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DENABLE_OPENCV=1
    BUILD_COMMAND make -j8
    )
set(LIBCAER_FOUND TRUE CACHE BOOL "Whether Libcaer was found.")
set(LIBCAER_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/libcaer/include" CACHE PATH "libcaer include dir")
set(LIBCAER_LIBRARY_DIRS "${CMAKE_BINARY_DIR}/libcaer/lib" CACHE PATH "libcaer lib dir")
set(LIBCAER_LIBRARIES "${CMAKE_SHARED_LIBRARY_PREFIX}caer${CMAKE_SHARED_LIBRARY_SUFFIX}" CACHE FILEPATH "libcaer lib file(s)")
include_directories(${LIBCAER_INCLUDE_DIRS})
link_directories(${LIBCAER_LIBRARY_DIRS})
list(APPEND 3RD_PARTY_LIBS ${LIBCAER_LIBRARIES})

#------------------------------------------------------------

# COMPILER FLAGS---------------------------------------------
set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -Ofast -O3 -pthread") # Added -pthread
set(GCC_COVERAGE_LINK_FLAGS    "")

list(APPEND CMAKE_CXX_FLAGS ${GCC_COVERAGE_COMPILE_FLAGS} )
list(APPEND CMAKE_EXE_LINKER_FLAGS ${GCC_COVERAGE_LINK_FLAGS})
#------------------------------------------------------------

# CXX & HPP--------------------------------------------------
file(GLOB_RECURSE COMP_TARGET_CXX ${CMAKE_CURRENT_SOURCE_DIR}/src/*)
file(GLOB_RECURSE COMP_TARGET_HPP ${CMAKE_CURRENT_SOURCE_DIR}/include/*)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
#------------------------------------------------------------

# CREATE THE TARGET EXECUTABLE-------------------------------
set(COMP_TARGET "semester-project")
add_executable( ${COMP_TARGET}
                ${COMP_TARGET_CXX}
                ${COMP_TARGET_HPP}
               )
target_link_libraries(${COMP_TARGET}
                      ${3RD_PARTY_LIBS}
                     )
#------------------------------------------------------------

# INSTALATION------------------------------------------------
set(EXECUTABLE_INSTALL_PATH ${CMAKE_CURRENT_LIST_DIR}/bin/${CMAKE_BUILD_TYPE})
#install(TARGETS ${CMAKE_CURRENT_LIST_DIR}/misc/99-.rules
#        DESTINATION /etc/udev/rules.d/)
install(TARGETS ${COMP_TARGET}
        DESTINATION ${EXECUTABLE_INSTALL_PATH}
       )
message(STATUS ${EXECUTABLE_INSTALL_PATH})
#------------------------------------------------------------
