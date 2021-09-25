cmake_minimum_required(VERSION 3.10.0)
cmake_policy(SET CMP0074 NEW)

project(cwipc_util)

include(CTest)
include(GNUInstallDirs)

# Ensure all executable, dll/so and link-library outputs end up in the same directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})


find_package(PCL 1.10 COMPONENTS common io octree filters features)
find_package(glfw3 3.2)
if(NOT glfw3_FOUND)
	add_subdirectory("third-party/glfw")
	include_directories("third-party/glfw/include")
endif()
find_package(OpenGL REQUIRED)

add_subdirectory("src")
add_subdirectory("apps/cwipc_ply2dump")
add_subdirectory("apps/cwipc_ply2dump_c")
add_subdirectory("apps/cwipc_pcl2dump")
add_subdirectory("apps/cwipc_dump2ply")
add_subdirectory("apps/cwipc_generate")
add_subdirectory("apps/cwipc_viewsynthetic")
add_subdirectory("apps/cwipc_tilefilter")
add_subdirectory("apps/cwipc_downsample")
add_subdirectory("apps/cwipc_remove_outliers")
add_subdirectory("python")
add_subdirectory("tests")
