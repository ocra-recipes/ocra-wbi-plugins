# Copyright: (C) 2014
# Authors: ...
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})

add_subdirectory(robots)

