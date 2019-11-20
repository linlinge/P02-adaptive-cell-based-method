cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(3D_viewer)
set(CMAKE_BUILD_TYPE "Debug")
find_package(PCL 1.2 REQUIRED)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
        message("OPENMP FOUND")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Opencv
find_package(OpenCV REQUIRED)

# glib
include_directories(glib)
include_directories(.)
add_subdirectory(glib)

# EXECUTABLE
add_executable (3D_viewer main.cpp SurfaceFitting.cpp HierachicalOutlierRemoval.cpp)
# PCL
target_link_libraries (3D_viewer ${PCL_LIBRARIES})
# GLIB
target_link_libraries(3D_viewer GLIB)
# OpenCV
target_link_libraries(3D_viewer ${OpenCV_LIBS})