#edit the following line to add the librarie's header files
FIND_PATH(tos_supervoxels_INCLUDE_DIR tos_supervoxels.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(tos_supervoxels_LIBRARY
    NAMES tos_supervoxels
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (tos_supervoxels_INCLUDE_DIR AND tos_supervoxels_LIBRARY)
   SET(tos_supervoxels_FOUND TRUE)
ENDIF (tos_supervoxels_INCLUDE_DIR AND tos_supervoxels_LIBRARY)

IF (tos_supervoxels_FOUND)
   IF (NOT tos_supervoxels_FIND_QUIETLY)
      MESSAGE(STATUS "Found tos_supervoxels: ${tos_supervoxels_LIBRARY}")
   ENDIF (NOT tos_supervoxels_FIND_QUIETLY)
ELSE (tos_supervoxels_FOUND)
   IF (tos_supervoxels_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find tos_supervoxels")
   ENDIF (tos_supervoxels_FIND_REQUIRED)
ENDIF (tos_supervoxels_FOUND)

