FIND_PACKAGE( PCL 1.8 REQUIRED )
FIND_PACKAGE( OpenCV REQUIRED )

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

# create an example application
ADD_EXECUTABLE(tabletop_object_detection_test tabletop_object_detection_test.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(tabletop_object_detection_test tabletop_object_detection ${PCL_LIBRARIES} ${OpenCV_LIBS})
