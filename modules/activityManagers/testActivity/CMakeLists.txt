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

# Check for SMLT
find_package(SMLT QUIET)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
    # Set preprocessor macro "USING_SMLT" to true.
    add_definitions( -DUSING_SMLT=1 )
ELSE()
    message("-- SMLT lib NOT found. Removing TaskOptimization sequence from source list.")
    # Set preprocessor macro "USING_SMLT" to false.
    add_definitions( -DUSING_SMLT=0 )
ENDIF()

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

LIST(APPEND link_libs   ${YARP_LIBRARIES}
                        ${WOCRA_LIBRARIES}
                        ${QUADPROG_LIBRARIES}
                        ${OCRA_LIBRARIES}
                        )

IF(SMLT_FOUND)
    LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${link_libs})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
