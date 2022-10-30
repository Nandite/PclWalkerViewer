cmake_minimum_required(VERSION 3.14)
project(PclWalkerViewer VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEBUG_FLAGS "-g -ggdb -O0 -pthread -pedantic")
set(RELEASE_FLAGS "-O2 -pthread -pedantic")

add_compile_options(-Wall -Wextra)

include_directories(include)
set(PROJECT_SOURCES include
        src/PclWalkerViewer.cpp
        src/Backtrace.cpp)

# PCL Package ##############################################################
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
############################################################################

# Boost Package ############################################################
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
############################################################################

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(pcl_walker_viewer ${PROJECT_SOURCES})
target_link_libraries(pcl_walker_viewer PRIVATE
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${CMAKE_DL_LIBS}
        Threads::Threads)

install(TARGETS pcl_walker_viewer DESTINATION /usr/local/bin)