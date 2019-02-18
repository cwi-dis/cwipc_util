cmake_minimum_required(VERSION 3.10.0)

project(cwipc_util)

include(CTest)
include(GNUInstallDirs)

find_package(PCL 1.8 REQUIRED COMPONENTS common io octree)

add_subdirectory("src")
add_subdirectory("apps/cwipc_ply2dump")
add_subdirectory("apps/cwipc_ply2dump_c")
add_subdirectory("apps/cwipc_pcl2dump")
add_subdirectory("apps/cwipc_dump2ply")
