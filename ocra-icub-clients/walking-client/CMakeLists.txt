# This file is part of walking-client.
# Copyright (C) Institut des Systèmes Intelligents et de Robotique (ISIR)
# author(s): Jorhabib Eljaik Gómez
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

# Make sure we are working with at least CMake 2.8.12
cmake_minimum_required(VERSION 2.8.12)

# Initiate the project
PROJECT(walking-client CXX)

# Append GUROBI's FindGUROBI.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/ocra-icub-clients/walking-client/cmake/")

# Make sure you have a C++11 compatible compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
 endif()

# Remember to export the following environmental variables when using GUROBI
# export GUROBI_HOME="/Library/gurobi701/mac64"
# export PATH="${PATH}:${GUROBI_HOME}/bin"
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${GUROBI_HOME}/lib
# export GRB_LICENSE_FILE=/Users/jorhabibeljaik/gurobi.lic
# Find Gurobi
find_package(GUROBI REQUIRED)
find_package(EigenGurobi REQUIRED)
# Set the project version.
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

# Add some helpful CMake functions
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)


# Get all of the source and header files.
file(GLOB folder_source src/*.cpp)
file(GLOB_RECURSE folder_header include/${PROJECT_NAME}/*.h )
file(GLOB folder_header_constraints include/${PROJECT_NAME}/constraints/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Headers Client" FILES ${folder_header})
source_group("Headers Constraints" FILES ${folder_header_constraints})

# Tell the compiler where to look for all other headers
include_directories(
${PROJECT_SOURCE_DIR}/include
${YARP_INCLUDE_DIRS}
${OcraIcub_INCLUDE_DIRS}
${OcraRecipes_INCLUDE_DIRS}
${GUROBI_INCLUDE_DIRS}
${EIGENGUROBI_INCLUDE_DIR}
)

# Add the client executable (binary)
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

message("GUROBI libraries: " ${GUROBI_LIBRARIES})
# Link to the appropriate libs
target_link_libraries(
${PROJECT_NAME}
${YARP_LIBRARIES}
${OcraRecipes_LIBRARIES}
ocra-icub
${GUROBI_LIBRARIES}
${EIGENGUROBI_LIBRARIES}
)

# Install to the bin/ directory if installed.
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
