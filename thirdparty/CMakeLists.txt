cmake_minimum_required(VERSION 3.10)

# imgui lib
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
add_subdirectory(robin-map)

option(TBB_TEST "Enable testing" OFF)
add_subdirectory(oneTBB)


