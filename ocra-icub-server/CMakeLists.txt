# This file is part of ocra-icub.
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

SET(PROJECTNAME ocra-icub-server)
PROJECT(${PROJECTNAME} CXX)

FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}/include
${OcraRecipes_INCLUDE_DIRS}
${OcraIcub_INCLUDE_DIRS}
)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(
${PROJECTNAME}
ocra-icub
)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
