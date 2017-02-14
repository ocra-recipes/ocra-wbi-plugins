# Looks for eigen-gurobi
# https://github.com/jeljaik/eigen-gurobi
#
# This cmake module will define the following variables
#  EIGENGUROBI_FOUND - eigen-gurobi was found in the system
#  EIGENGUROBI_INCLUDE_DIR - The eigen-gurobi include directories
#  EIGENGUROBI_LIBRARIES - The libraries needed to use eigen-gurobi

find_path(EIGENGUROBI_INCLUDE_DIR
          NAMES Gurobi.h eigen_gurobi_api.h
          PATHS "$ENV{EIGENGUROBI_HOME}/include"
                "/usr/local/include/eigen-gurobi")
find_library(EIGENGUROBI_LIBRARIES
             NAMES libeigen-gurobi.dylib
           PATHS "$ENV{EIGENGUROBI_HOME}/build/lib"
         "/usr/local/lib")
set(EIGENGUROBI_FOUND true)
message("-- EIGENGUROBI_INCLUDE_DIR: " ${EIGENGUROBI_INCLUDE_DIR})
message("-- EIGENGUROBI_LIBRARIES: " ${EIGENGUROBI_LIBRARIES})

mark_as_advanced(EIGENGUROBI_INCLUDE_DIR EIGENGUROBI_LIBRARIES)
