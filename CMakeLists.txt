cmake_minimum_required(VERSION 2.8.11)
project(ocra-wbi-plugins CXX)

set(OCRA_WBI_PLUGINS_MAJOR_VERSION 0)
set(OCRA_WBI_PLUGINS_MINOR_VERSION 0)
set(OCRA_WBI_PLUGINS_PATCH_VERSION 1)
set(OCRA_WBI_PLUGINS_VERSION ${OCRA_WBI_PLUGINS_MAJOR_VERSION}.${OCRA_WBI_PLUGINS_MINOR_VERSION}.${OCRA_WBI_PLUGINS_PATCH_VERSION})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(YCM REQUIRED)

#add yarp definitions
add_definitions(${YARP_DEFINES}) #this contains also -D_REENTRANT

# include YARP CMake modules
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${YCM_MODULE_PATH} ${CMAKE_MODULE_PATH})

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

add_subdirectory(modules)

include(AddUninstallTarget)



