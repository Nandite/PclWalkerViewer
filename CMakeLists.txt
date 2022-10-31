cmake_minimum_required(VERSION 3.14)
project(PclWalkerViewer VERSION 1.0 LANGUAGES CXX)

set(TARGET_NAME pcl_walker_viewer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEBUG_FLAGS "-g -ggdb -O0 -pthread -pedantic")
set(RELEASE_FLAGS "-O2 -pthread -pedantic")

add_compile_options(-Wall -Wextra)

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

# Thread Package ###########################################################
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
############################################################################

include_directories(include)
set(PROJECT_SOURCES include src/PclWalkerViewer.cpp)
set(PROJECT_LIBRARIES ${PCL_LIBRARIES} ${Boost_LIBRARIES} Threads::Threads)

add_executable(${TARGET_NAME} ${PROJECT_SOURCES})
target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_LIBRARIES})

install(TARGETS ${TARGET_NAME} DESTINATION /usr/local/bin)