
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_dump2ply cwipc_dump2ply.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_dump2ply cwipc_util ${PCL_LIBRARIES})

install(TARGETS cwipc_dump2ply RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_test(NAME cwipc_dump2ply COMMAND cwipc_dump2ply ${CMAKE_SOURCE_DIR}/tests/fixtures/input/pcl_frame1.ply ./pcl_frame1.cwipcdump )
