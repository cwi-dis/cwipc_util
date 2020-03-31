cmake_minimum_required(VERSION 3.10.0)

add_library(cwipc_util SHARED cwipc_util.cpp cwipc_synthetic.cpp cwipc_window.cpp window_util.cpp)

target_sources(cwipc_util
	PRIVATE
	"../include/cwipc_util/api.h"
	"../include/cwipc_util/api_pcl.h"
	"window_util.hpp"
)

target_include_directories(
    cwipc_util PUBLIC
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)
target_include_directories(
    cwipc_util PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${glfw_INCLUDE_DIRS}
)

target_link_libraries(cwipc_util PRIVATE ${PCL_LIBRARIES} glfw OpenGL::GL)

install(TARGETS cwipc_util
    EXPORT cwipc_util
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cwipc_util"
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_util
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_util-config.cmake DESTINATION lib/cmake/cwipc_util)

install(EXPORT cwipc_util DESTINATION lib/cmake/cwipc_util)
