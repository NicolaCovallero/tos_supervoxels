#edit the following line to add the librarie's header files
FIND_PATH(tabletop_object_detection_INCLUDE_DIR tabletop_object_detection.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(tabletop_object_detection_LIBRARY
    NAMES tabletop_object_detection
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (tabletop_object_detection_INCLUDE_DIR AND tabletop_object_detection_LIBRARY)
   SET(tabletop_object_detection_FOUND TRUE)
ENDIF (tabletop_object_detection_INCLUDE_DIR AND tabletop_object_detection_LIBRARY)

IF (tabletop_object_detection_FOUND)
   IF (NOT tabletop_object_detection_FIND_QUIETLY)
      MESSAGE(STATUS "Found tabletop_object_detection: ${tabletop_object_detection_LIBRARY}")
   ENDIF (NOT tabletop_object_detection_FIND_QUIETLY)
ELSE (tabletop_object_detection_FOUND)
   IF (tabletop_object_detection_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find tabletop_object_detection")
   ENDIF (tabletop_object_detection_FIND_REQUIRED)
ENDIF (tabletop_object_detection_FOUND)

