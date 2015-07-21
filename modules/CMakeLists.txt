add_subdirectory(ISIRWholeBodyController)

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_GOCRA gOcra)

IF(${PC_GOCRA_FOUND})
    message("Found gOcra libs. Adding gOcraController build.")
    add_subdirectory(gOcraController)
ELSE()
    message("Did NOT find gOcra libs. Skipping gOcraController build.")
ENDIF()


add_subdirectory(ISIRObservers)
