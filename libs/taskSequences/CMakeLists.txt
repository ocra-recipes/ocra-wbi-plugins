# This file is part of ocra-yarp.
# Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
# author(s): Ryan Lober, Antoine Hoarau
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

SET(PROJECTNAME taskSequences)
PROJECT(${PROJECTNAME} CXX)

find_package(OcraRecipes REQUIRED)

FILE(GLOB_RECURSE folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

FILE(GLOB_RECURSE folder_header ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECTNAME}/*.h)

# Check for SMLT
find_package(SMLT QUIET)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
    # Set preprocessor macro "USING_SMLT" to true.
    add_definitions( -DUSING_SMLT=1 )
ELSE()
    message("-- SMLT lib NOT found. Removing TaskOptimization sequence from source list.")
    list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/TaskOptimization.cpp)
    list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/StandingReach.cpp)
    list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/MoveWeight.cpp)
    list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/Experiment.cpp)
    # Set preprocessor macro "USING_SMLT" to false.
    add_definitions( -DUSING_SMLT=0 )
    #set_property(TARGET ${liddddbrary_name} APPEND PROPERTY COMPILE_DEFINITIONS USING_SMLT=0 XENOMAI=1)
ENDIF()


SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${OcraRecipes_INCLUDE_DIRS}
                    )


ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source} ${folder_header})

LIST(APPEND link_libs   #${YARP_LIBRARIES}
                        #${ICUB_LIBRARIES}
                        ${iDynTree_LIBRARIES}
                        ${yarpWholeBodyInterface_LIBRARIES}
                        ${OcraRecipes_LIBRARIES}
)

IF(SMLT_FOUND)
    LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${link_libs})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME} DESTINATION include)
