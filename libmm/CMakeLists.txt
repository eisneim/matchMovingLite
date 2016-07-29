cmake_minimum_required(VERSION 2.8)

set(MM_LIB_SOURCES mmCommon.hpp                mmCommon.cpp
                    MM.hpp                      MM.cpp
    )

add_library(libmmLibrary ${MM_LIB_SOURCES} )