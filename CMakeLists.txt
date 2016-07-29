cmake_minimum_required (VERSION 2.8)
set (CMAKE_CXX_STANDARD 14)
# add_definitions("-std=gnu++11")

#this is the name of the project
PROJECT( matchMovingLite )

# add_subdirectory( src )

# this is for recursively searching for other directories
# add_subdirectory (src)

# ==========================================================================
#             recursively include all files under src
# ==========================================================================
file(GLOB_RECURSE SRC_SOURCES "src/*.cpp")
file(GLOB_RECURSE SRC_HEADERS "src/*.h")

set (SRC_INCLUDE_DIRS "")
foreach (_headerFile ${SRC_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND SRC_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES SRC_INCLUDE_DIRS)
# ==========================================================================

# source to include
# include_directories(.)
include_directories(${SRC_INCLUDE_DIRS})
include_directories("libmm")


add_subdirectory("libmm")

FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( mm main.cpp ${SRC_SOURCES})

TARGET_LINK_LIBRARIES( mm libmmLibrary ${OpenCV_LIBS} )
