cmake_minimum_required(VERSION 3.16)
project(RANSAC2)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ".")
set(CMAKE_CXX_FLAGS "-fPIC -O3 -ffast-math -msse3 -mfpmath=sse")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)

foreach(dep math;random;gui;maps)
    if (NOT TARGET mrpt::${dep})
        find_package(mrpt-${dep} REQUIRED)
    endif()
endforeach()

enable_testing()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( EIGEN3_INCLUDE_DIR )


include(CheckCXXCompilerFlag)
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))

    add_compile_options(
            -Werror
            -Wall
            -Wextra
            -Wpedantic
            -Wcast-align
            -Wcast-qual
            -Wconversion
            -Wctor-dtor-privacy
            -Wenum-compare
            -Wfloat-equal
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Woverloaded-virtual
            -Wredundant-decls
            -Wsign-conversion
            -Wsign-promo
    )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    check_cxx_compiler_flag(-std=c++17 SUPPORTS_CXX17)

    if(SUPPORTS_CXX17)
        add_compile_options(-std=c++17)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++17")
    endif()
endif()

add_executable(RANSAC2 main.cpp
        Bird_view.cpp
        container.cpp
        HSV.cpp
        Other_func.cpp
        Ransac.cpp
        Show_all.cpp
        simple_line_to_polygon.cpp
        dashed_line.cpp
        draw.cpp
        distance_to_lane.cpp
        settings.cpp
        ../include/Ransac.h
)

set(PACKAGE_STRING RANSAC2)
target_link_libraries(RANSAC2 ${OpenCV_LIBS} )
target_link_libraries (RANSAC2 Eigen3::Eigen)
foreach(dep math;random;gui;maps)
    target_link_libraries(RANSAC2 mrpt::${dep})
endforeach()
