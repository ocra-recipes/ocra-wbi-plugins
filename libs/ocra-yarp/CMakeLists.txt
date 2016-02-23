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

SET(PROJECTNAME ocra-yarp)
PROJECT(${PROJECTNAME} CXX)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${taskSequences_SOURCE_DIR}/include
                    )

ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${yarpWholeBodyInterface_LIBRARIES}
                                     ${OcraRecipes_LIBRARIES}
                                     ${Boost_LIBRARIES}
                                     taskSequences
                                     )


install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ${PROJECTNAME}
        EXPORT OcraYarp
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECTNAME})
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
