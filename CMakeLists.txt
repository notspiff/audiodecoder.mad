project(audiodecoder.mad)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

enable_language(CXX)

find_package(xbmc REQUIRED)
find_package(MAD REQUIRED)

include_directories(${XBMC_INCLUDE_DIR}
                    ${MAD_INCLUDE_DIRS})


set(MAD_SOURCES src/MP3Codec.cpp)

build_addon(audiodecoder.mad MAD DEPLIBS)
