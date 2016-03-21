cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(iros_bshot)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#For enabling SSE4_1 for faster Hamming distance computation 
#Need to check if this is making the computations faster...
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DNDEBUG -O3 -msse -msse2 -msse3 -msse4.1 -msse4.2")

# Adding PCL library
#set(PCL_DIR "/home/sai/workspace/pcl-pcl-1.7.2/build")
find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_INCLUDE_LIBS})
add_definitions(${PCL_DEFINITIONS})

add_executable (shot src/shot.cpp)
target_link_libraries (shot ${PCL_LIBRARIES} )

add_executable (bshot src/bshot.cpp)
target_link_libraries (bshot ${PCL_LIBRARIES} )

