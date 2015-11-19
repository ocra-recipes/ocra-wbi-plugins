SET(PROJECTNAME taskSequences)
PROJECT(${PROJECTNAME} CXX)

find_package(WOCRA REQUIRED)

FILE(GLOB_RECURSE folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

FILE(GLOB_RECURSE folder_header ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECTNAME}/*.h)

# Check for SMLT
find_package(SMLT)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(SMLT_INCLUDE_DIRS)
    # Set preprocessor macro "USING_SMLT" to true.
    add_definitions( -DUSING_SMLT=1 )
ELSE()
    message("-- SMLT lib NOT found. Removing TaskOptimization sequence from source list.")
    list(REMOVE_ITEM folder_source ${CMAKE_CURRENT_SOURCE_DIR}/src/sequences/TaskOptimization.cpp)
    # Set preprocessor macro "USING_SMLT" to false.
    add_definitions( -DUSING_SMLT=0 )
    #set_property(TARGET ${liddddbrary_name} APPEND PROPERTY COMPILE_DEFINITIONS USING_SMLT=0 XENOMAI=1)
ENDIF()


SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${WOCRA_INCLUDE_DIRS}
                    ${ocraWbiPlugins_SOURCE_DIR}/include
                    )


ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source})

LIST(APPEND link_libs   ${YARP_LIBRARIES}
                        ${ICUB_LIBRARIES}
                        ${iDynTree_LIBRARIES}
                        ${yarpWholeBodyInterface_LIBRARIES}
                        ${WOCRA_LIBRARIES}
                        ocraWbiPlugins)

IF(SMLT_FOUND)
    LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${link_libs} ryan_sucks)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECTNAME} DESTINATION include)
