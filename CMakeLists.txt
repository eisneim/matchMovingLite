cmake_minimum_required (VERSION 2.8)
set (CMAKE_CXX_STANDARD 14)

#this is the name of the project
PROJECT( matchMovingLite )


# this is for recursively searching for other directories
# add_subdirectory (src)

file(GLOB_RECURSE Foo_SOURCES "src/*.cpp")
file(GLOB_RECURSE Foo_HEADERS "src/*.h")

set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)

# source to include
# include_directories(.)
include_directories(${Foo_INCLUDE_DIRS})

FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( mm main.cpp ${Foo_SOURCES})

TARGET_LINK_LIBRARIES( mm ${OpenCV_LIBS} )
