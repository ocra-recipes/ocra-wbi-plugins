# Copyright (C) 2014 CoDyCo
# Author: Darwin Lau, MingXing Liu, Ryan Lober
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8.11)
SET(PROJECTNAME gOcraController)
PROJECT(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "-fpermissive")

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${YCM_MODULE_PATH} ${CMAKE_MODULE_PATH})


find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(modHelp REQUIRED)

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_GOCRA gOcra REQUIRED)
PKG_CHECK_MODULES(PC_OCRA_FRAMEWORK ocra_framework REQUIRED)

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/gOcraController/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_DIRECTORIES(${PC_GOCRA_LIBRARY_DIRS}
                 ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS}
                 )

#ADD_LIBRARY(orcWbiModel SHARED src/orcWbiModel.cpp)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    # ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${PC_EIGEN_INCLUDE_DIRS}
                    
                    ${BOOST_INCLUDE_DIRS}
                    ${PC_GOCRA_INCLUDE_DIRS}
                    ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS}
                    ${modHelp_INCLUDE_DIRS}/include
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${modHelp_LIBRARIES}
                                     ${PC_GOCRA_LIBRARIES}
                                     ${PC_OCRA_FRAMEWORK_LIBRARIES}
                                     )


INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
