FIND_PACKAGE(PCL 1.8 REQUIRED)

# driver source files
SET(sources tos_supervoxels.cpp)
# application header files
SET(headers tos_supervoxels.h)
# locate the necessary dependencies
# add the necessary include directories
INCLUDE_DIRECTORIES(.)

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

# create the shared library
ADD_LIBRARY(tos_supervoxels SHARED ${sources})
TARGET_LINK_LIBRARIES(tos_supervoxels ${PCL_LIBRARIES}) 

# link necessary libraries
INSTALL(TARGETS tos_supervoxels
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
INSTALL(FILES ${headers} DESTINATION include/iridrivers)
INSTALL(FILES ../Findtos_supervoxels.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
