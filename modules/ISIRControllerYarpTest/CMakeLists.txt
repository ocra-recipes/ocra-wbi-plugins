# Copyright (C) 2013 CoDyCo Consortium
# Author: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME ISIRControllerYarpTest)
PROJECT(${PROJECTNAME} CXX)


#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


# find packages
#--------------------------

find_package(YARP)
find_package(ICUB)
find_package(YCM)
# now stuff related to ISIR_controller
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(EIGEN eigen_lgsm REQUIRED)
PKG_CHECK_MODULES(PC_ORCISIR_ISIRCONTROLLER orcisir_ISIRController REQUIRED)
PKG_CHECK_MODULES(PC_ORC_FRAMEWORK orc_framework REQUIRED)
PKG_CHECK_MODULES(PC_MODHELP modHelp REQUIRED)
if(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)
    message("Failed to detect orcisir_ISIRController, check PKG_CONFIG_PATH." )
endif(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)

# include directories
#--------------------------

# these are necessary and taken from $CODYCO_ROOT/extern
#SET(${iDyn_INCLUDE_DIRS} /home/shak/compil/samaxe-64/include)
				
INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${PC_MODHELP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${BOOST_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
                    ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}
                    ${PC_ORC_FRAMEWORK_INCLUDE_DIRS}                  
                    )
                    				
# link libraries
#--------------------------

LINK_DIRECTORIES(${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS}
    ${PC_ORC_FRAMEWORK_LIBRARY_DIRS}
                 ${PC_MODHELP_LIBRARY_DIRS})

#this must go always before link_directories
ADD_LIBRARY(iCubModel SHARED src/iCubModel.cpp)
ADD_EXECUTABLE(${PROJECTNAME} src/main.cpp)


TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     icubmod
                                     iCubModel
                                     ${PC_MODHELP_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
                                     ${PC_ORC_FRAMEWORK_LIBRARIES}
                                     )
                                     




