SET(PROJECTNAME taskSequences)
PROJECT(${PROJECTNAME} CXX)

find_package(WOCRA REQUIRED)

FILE(GLOB_RECURSE folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# FILE(GLOB folder_source_seq ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/*.cpp)

FILE(GLOB_RECURSE folder_header ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECTNAME}/*.h)
# FILE(GLOB folder_header_seq ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECTNAME}/sequences/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})# ${folder_header_seq})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${WOCRA_INCLUDE_DIRS}
                    ${ocraWbiPlugins_SOURCE_DIR}/include
                    )

ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${WOCRA_LIBRARIES}
                                     ocraWbiPlugins
                                     )


INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME}/sequences DESTINATION include/sequences)
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME} DESTINATION include)
