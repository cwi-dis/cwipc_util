cmake_minimum_required(VERSION 3.16.0)

add_library(cwipc_util SHARED cwipc_util.cpp cwipc_synthetic.cpp cwipc_capturer.cpp cwipc_filters.cpp cwipc_window.cpp window_util.cpp cwipc_certh.cpp cwipc_proxy.cpp )

target_sources(cwipc_util
	PRIVATE
	"../include/cwipc_util/api.h"
	"../include/cwipc_util/api_pcl.h"
	"../include/cwipc_util/internal.h"
	"window_util.hpp"
)

target_include_directories(
    cwipc_util PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)
target_include_directories(
    cwipc_util PRIVATE
    ${glfw_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	target_link_directories(
		cwipc_util PUBLIC
		${PCL_LIBRARY_DIRS}
	)
	target_compile_definitions(
		cwipc_util PRIVATE
		${PCL_DEFINITIONS}
	)
endif()
target_link_libraries(cwipc_util PUBLIC ${PCL_LIBRARIES})
target_link_libraries(cwipc_util PRIVATE glfw OpenGL::GL OpenGL::GLU)
target_link_libraries(cwipc_util PRIVATE nlohmann_json::nlohmann_json)

if(WIN32)
	target_link_libraries(cwipc_util PRIVATE wsock32 ws2_32)
endif()

install(TARGETS cwipc_util
    EXPORT cwipc_util
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION include/cwipc_util
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_util
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_util-config.cmake DESTINATION lib/cmake/cwipc_util)

install(EXPORT cwipc_util DESTINATION lib/cmake/cwipc_util)

if(WIN32)
	install(FILES $<TARGET_PDB_FILE:cwipc_util> DESTINATION ${CMAKE_INSTALL_BINDIR} OPTIONAL)
endif()
