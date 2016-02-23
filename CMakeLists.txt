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

cmake_minimum_required(VERSION 2.8.11)
project(ocra-wbi CXX)

set(OCRA_WBI_MAJOR_VERSION 0)
set(OCRA_WBI_MINOR_VERSION 0)
set(OCRA_WBI_PATCH_VERSION 1)
set(OCRA_WBI_VERSION ${OCRA_WBI_MAJOR_VERSION}.${OCRA_WBI_MINOR_VERSION}.${OCRA_WBI_PATCH_VERSION})

option(GENERATE_DOCUMENTATION "Generate the doxygen documentation." TRUE)


find_package(ICUB REQUIRED)
find_package(YCM REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

list(APPEND CMAKE_MODULE_PATH
        ${YARP_MODULE_PATH}
        ${YCM_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules
     )

find_package(OcraRecipes REQUIRED)


include_directories(AFTER
                    ${OcraRecipes_INCLUDE_DIRS}
                    )


#add yarp definitions
add_definitions(${YARP_DEFINES}) #this contains also -D_REENTRANT

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)


add_subdirectory(libs)
add_subdirectory(modules)
add_subdirectory(bin)



include(AddUninstallTarget)

# add a target to generate API documentation with Doxygen
if(GENERATE_DOCUMENTATION)
    find_package(Doxygen REQUIRED)
    message("Generating API documentation with Doxygen")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(
        docs
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(GENERATE_DOCUMENTATION)
