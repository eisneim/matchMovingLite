cmake_minimum_required(VERSION 2.8)

set(MM_LIB_SOURCES mmCommon.hpp                mmCommon.cpp
                    MM.hpp                      MM.cpp
                    mmFeature.hpp               mmFeature.cpp
                    mmBA.hpp                    mmBA.cpp
                    mmStereoUtils.hpp           mmStereoUtils.cpp
    )



# message("should include opencv: ${OpenCV_LIBS}" )

add_library(libmmLibrary ${MM_LIB_SOURCES} )

target_link_libraries(libmmLibrary ${OpenCV_LIBS})