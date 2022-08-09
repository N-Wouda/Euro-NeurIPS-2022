cmake_minimum_required(VERSION 3.2)

project(hgs_vrptw
        VERSION 1.0.0
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_definitions(-Werror -Wall -Wextra -Wpedantic -fPIC)

if (CMAKE_BUILD_TYPE MATCHES RELEASE)
    add_definitions(-O3)
endif ()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include)

add_subdirectory(extern)

add_subdirectory(src)
add_subdirectory(test)
