# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8.11)
SET(PROJECTNAME basicWholeBodyControlModule)
PROJECT(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "-fpermissive")

find_package(YARP REQUIRED)
find_package(YCM REQUIRED) 

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${YCM_MODULE_PATH} ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)


find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(EIGEN eigen_lgsm REQUIRED)
PKG_CHECK_MODULES(PC_ORCISIR_ISIRCONTROLLER orcisir_ISIRController REQUIRED)
PKG_CHECK_MODULES(PC_ORC_FRAMEWORK orc_framework REQUIRED)
PKG_CHECK_MODULES(PC_MODHELP modHelp REQUIRED)

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./src/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_DIRECTORIES(${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS}
                 ${PC_ORC_FRAMEWORK_LIBRARY_DIRS}
                 ${PC_MODHELP_LIBRARY_DIRS}
                 )

#ADD_LIBRARY(orcWbiModel SHARED src/orcWbiModel.cpp)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src
                    ${YARP_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${BOOST_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
                    ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}
                    ${PC_ORC_FRAMEWORK_INCLUDE_DIRS} 
                    ${PC_MODHELP_INCLUDE_DIRS}
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
                                     ${PC_ORC_FRAMEWORK_LIBRARIES}
                                     ${PC_MODHELP_LIBRARIES}
                                     )


INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
