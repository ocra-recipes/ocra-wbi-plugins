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

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install( FILES ${scripts}
DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/..)

add_subdirectory(robots)
