cmake_minimum_required (VERSION 2.8)

#this is the name of the project
PROJECT( matchMovingLite )

# this is for recursively searching for other directories
# add_subdirectory (DirectoryName)

# source to include
include_directories(.)
FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( matchMove main.cpp )
TARGET_LINK_LIBRARIES( main ${OpenCV_LIBS} )
