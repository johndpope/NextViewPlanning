cmake_minimum_required(VERSION 2.9)
project(NextViewPlanning)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH
        ${CMAKE_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/CMake
        ${CMAKE_MODULE_PATH}
        )
#message("CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
#message("CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}")

find_package(OpenMesh REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIR})
link_directories(${OPENMESH_LIBRARY_DIR})

#message(${OPENMESH_FOUND})
#message(${OPENMESH_LIBRARIES})

include_directories(src)
include_directories(lib)

set(SOURCE_FILES src/main.cpp src/PointCloud.cpp src/converter.cpp src/utilities.cpp src/Camera.cpp)

add_library(nvp_library ${SOURCE_FILES})

add_executable(nextViewPlanning src/main.cpp)
target_link_libraries(nextViewPlanning nvp_library ${OPENMESH_LIBRARIES})

install (TARGETS nextViewPlanning DESTINATION ${PROJECT_SOURCE_DIR}/bin)
