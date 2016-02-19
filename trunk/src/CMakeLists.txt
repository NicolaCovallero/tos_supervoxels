FIND_PACKAGE(PCL 1.8 REQUIRED)

# driver source files
SET(sources tabletop_object_detection.cpp)
# application header files
SET(headers tabletop_object_detection.h)
# locate the necessary dependencies
# add the necessary include directories
INCLUDE_DIRECTORIES(.)

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

# create the shared library
ADD_LIBRARY(tabletop_object_detection SHARED ${sources})
TARGET_LINK_LIBRARIES(tabletop_object_detection ${PCL_LIBRARIES}) 

# link necessary libraries
INSTALL(TARGETS tabletop_object_detection
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
INSTALL(FILES ${headers} DESTINATION include/iridrivers)
INSTALL(FILES ../Findtabletop_object_detection.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
