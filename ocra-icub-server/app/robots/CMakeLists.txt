option(CODYCO_INSTALL_ALL_ROBOTS "Installation of all available robots" ON)

set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE PATH
    "Name of your robot")

set(ROBOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_NAME}")

# If no env variable YARP_ROBOT_NAME is defined
if(CODYCO_INSTALL_ALL_ROBOTS)
    set(robots icubGazeboSim
               iCubGenova02
               iCubGenova03
               iCubGenova02
               iCubParis02
               iCubParis01
               icubSim
               )
else()
  if(ROBOT_NAME)
      set(robots ${ROBOT_NAME})
  endif()
endif()



foreach(robot ${robots})
    yarp_install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${robot}
                 DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR})
endforeach()
