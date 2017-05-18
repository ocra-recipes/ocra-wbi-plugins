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

cmake_minimum_required(VERSION 2.8.11)
project(ocra-icub-clients CXX)


add_subdirectory(example-client)
add_subdirectory(stepping-demo)
add_subdirectory(task-operations-demo)
add_subdirectory(sitting-demo)
add_subdirectory(standing-demo)

find_package(GUROBI)
if (GUROBI_FOUND)
    if(NOT WIN32)
        string(ASCII 27 Esc)
    endif()
    message("${Esc}[1;35m  -- Gurobi was found in your system. adding walking-client! ${Esc}[m")
    add_subdirectory(walking-client)
endif()

# add_subdirectory(your-client)
