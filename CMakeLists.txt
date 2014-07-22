cmake_minimum_required(VERSION 2.8.11)
project(codyco-isir CXX)

set(CODYCO_ISIR_MAJOR_VERSION 0)
set(CODYCO_ISIR_MINOR_VERSION 0)
set(CODYCO_ISIR_PATCH_VERSION 1)
set(CODYCO_ISIR_VERSION ${CODYCO_ISIR_MAJOR_VERSION}.${CODYCO_ISIR_MINOR_VERSION}.${CODYCO_ISIR_PATCH_VERSION})

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

#add yarp definitions
add_definitions(${YARP_DEFINES}) #this contains also -D_REENTRANT

# include YARP CMake modules
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

add_subdirectory(modules)

include(AddUninstallTarget)



