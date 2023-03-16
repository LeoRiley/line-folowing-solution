cmake_minimum_required(VERSION 3.5)

project(my_project)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add include directories for OpenCV
include_directories(/opt/homebrew/Cellar/opencv/4.7.0_1/include/opencv4/opencv2/)

# Add executable
add_executable(main main.cpp)

# Link against OpenCV libraries
target_link_libraries(main ${OpenCV_LIBS})
