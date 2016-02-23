FIND_PACKAGE( PCL 1.8 REQUIRED )
FIND_PACKAGE( OpenCV REQUIRED )

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

# create an example application
ADD_EXECUTABLE(tos_supervoxels_test tos_supervoxels_test.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(tos_supervoxels_test tos_supervoxels ${PCL_LIBRARIES} ${OpenCV_LIBS})
