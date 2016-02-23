cmake_minimum_required(VERSION 2.8.11)
project(ocra-wbi CXX)

set(OCRA_WBI_MAJOR_VERSION 0)
set(OCRA_WBI_MINOR_VERSION 0)
set(OCRA_WBI_PATCH_VERSION 1)
set(OCRA_WBI_VERSION ${OCRA_WBI_MAJOR_VERSION}.${OCRA_WBI_MINOR_VERSION}.${OCRA_WBI_PATCH_VERSION})

find_package(ICUB REQUIRED)
find_package(YCM REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

list(APPEND CMAKE_MODULE_PATH
        ${YARP_MODULE_PATH}
        ${YCM_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules
     )

find_package(OcraRecipes REQUIRED)


include_directories(AFTER
                    ${OcraRecipes_INCLUDE_DIRS}
                    )


#add yarp definitions
add_definitions(${YARP_DEFINES}) #this contains also -D_REENTRANT

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)


add_subdirectory(libs)
add_subdirectory(modules)
add_subdirectory(bin)



include(AddUninstallTarget)
