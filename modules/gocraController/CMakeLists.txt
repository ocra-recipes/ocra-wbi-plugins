SET(PROJECTNAME gocraController)
PROJECT(${PROJECTNAME} CXX)

find_package(OCRA REQUIRED)
find_package(GOCRA REQUIRED)
find_package(QUADPROG REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${GOCRA_INCLUDE_DIRS}
                    ${ocraWbiPlugins_SOURCE_DIR}/include
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${GOCRA_LIBRARIES}
                                     ${OCRA_LIBRARIES}
                                     ${QUADPROG_LIBRARIES}
                                     ${Boost_LIBRARIES}
                                     ocraWbiPlugins
                                     )



INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
