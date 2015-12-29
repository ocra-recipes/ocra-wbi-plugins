SET(PROJECTNAME testActivity)
PROJECT(${PROJECTNAME} CXX)

find_package(WOCRA REQUIRED)
find_package(OCRA REQUIRED)
find_package(QUADPROG REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
                    ${PROJECT_SOURCE_DIR}/include
                    ${WOCRA_INCLUDE_DIRS}
                    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(  ${PROJECTNAME}
                        ${YARP_LIBRARIES}
                        ${WOCRA_LIBRARIES}
                        ${QUADPROG_LIBRARIES}
                        ${OCRA_LIBRARIES}
                        )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
