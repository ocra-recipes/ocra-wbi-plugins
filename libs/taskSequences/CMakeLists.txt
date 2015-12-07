# Copyright (C) 2014 CoDyCo
# Author: Darwin Lau, MingXing Liu, Ryan Lober
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8.11)
SET(PROJECTNAME taskSequences)
PROJECT(${PROJECTNAME})

list(APPEND CMAKE_CXX_FLAGS "-fpermissive")

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${YCM_MODULE_PATH} ${CMAKE_MODULE_PATH})


find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(modHelp REQUIRED)

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_WLQP_CONTROL wLQP-Control REQUIRED)
# PKG_CHECK_MODULES(PC_OCRA_FRAMEWORK ocra_framework REQUIRED)

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

FILE(GLOB_RECURSE folder_source ./src/*.cpp)
FILE(GLOB_RECURSE folder_header ./include/${PROJECTNAME}/*.h)

# Check for SMLT
find_package(SMLT)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
    # Set preprocessor macro "USING_SMLT" to true.
    add_definitions( -DUSING_SMLT=1 )
# ELSE()
#     message("-- SMLT lib NOT found. Removing TaskOptimization sequence from source list.")
#     list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/TaskOptimization.cpp)
#     # Set preprocessor macro "USING_SMLT" to false.
#     add_definitions( -DUSING_SMLT=0 )
#     #set_property(TARGET ${liddddbrary_name} APPEND PROPERTY COMPILE_DEFINITIONS USING_SMLT=0 XENOMAI=1)
ENDIF()


SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_DIRECTORIES(${PC_WLQP_CONTROL_LIBRARY_DIRS}
                #  ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS}
                ${ocraWbiPlugins_SOURCE_DIR}/src
                 )



INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    # ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${PC_EIGEN_INCLUDE_DIRS}

                    ${BOOST_INCLUDE_DIRS}
                    ${PC_WLQP_CONTROL_INCLUDE_DIRS}
                    ${ocraWbiPlugins_SOURCE_DIR}/include
                    # ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS}
                    ${modHelp_INCLUDE_DIRS}/include
                    )

ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source} ${folder_header})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${modHelp_LIBRARIES}
                                     ${PC_WLQP_CONTROL_LIBRARIES}
                                     ocraWbiPlugins
                                    #  ${PC_OCRA_FRAMEWORK_LIBRARIES}
                                     )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)

# INSTALL(FILES ${folder_header} DESTINATION include/${PROJECTNAME})
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME} DESTINATION include)



# INSTALL(TARGETS ${PROJECTNAME}
#         LIBRARY DESTINATION lib
#         PUBLIC_HEADER DESTINATION include)
