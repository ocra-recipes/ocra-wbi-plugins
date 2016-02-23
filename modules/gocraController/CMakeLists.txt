SET(PROJECTNAME gocraController)
PROJECT(${PROJECTNAME} CXX)

find_package(OcraRecipes REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${OcraRecipes_INCLUDE_DIRS}
                    ${ocra-yarp_SOURCE_DIR}/include
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${yarpWholeBodyInterface_LIBRARIES}
                                     ${OcraRecipes_LIBRARIES}
                                     ocra-yarp
                                     )



INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
