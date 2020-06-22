cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME prevention-dataset)
project(${PROJECT_NAME})

find_package(OpenCV REQUIRED)

if(OpenCV_VERSION VERSION_GREATER "3.0")
    MESSAGE( FATAL_ERROR "OpenCV < 4.0 is REQUIRED" )
else()
    MESSAGE ( STATUS " OpenCV libs: " ${OpenCV_LIBS} )
    MESSAGE ( STATUS " OpenCV includes: " ${OpenCV_INCLUDE_DIRS} )
endif()

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


link_directories(${OpenCV_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} 	     
                      ${OpenCV_LIBS})



