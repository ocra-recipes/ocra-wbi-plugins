SET(PROJECTNAME ocra-controller-server)
PROJECT(${PROJECTNAME} CXX)

find_package(OcraRecipes REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${OcraRecipes_INCLUDE_DIRS}
                    ${ocra-yarp_SOURCE_DIR}/include
                    ${taskSequences_SOURCE_DIR}/include
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} #${YARP_LIBRARIES}
                                     #${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${OcraRecipes_LIBRARIES}
                                     ${Boost_LIBRARIES}
                                     ocra-yarp
                                     )



INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
