cmake_minimum_required(VERSION 3.5)
project(astar)
add_compile_options(-std=c++11)


set (EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
  "Target directory for all executables.")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)
# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

    


add_executable(astar_cv sample/astar_cv.cpp)
target_link_libraries(astar_cv ${OpenCV_LIBS} ${Eigen_LIBS})

add_executable(astar sample/astar.cpp)
target_link_libraries(astar ${Eigen_LIBS})
