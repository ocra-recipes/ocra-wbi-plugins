SET(PROJECTNAME ocraObservers)
PROJECT(${PROJECTNAME})

find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(WOCRA REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

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


INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME} DESTINATION include)
