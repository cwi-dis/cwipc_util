cmake_minimum_required(VERSION 3.10.0)

find_package(Python3 COMPONENTS Interpreter)

# Unfortunately Ubuntu 18.04 doesn't have the Python3 package (introduced in cmake 3.12 or so). Work around it.
if(NOT Python3_FOUND)
    find_program(PY3INTERPRETER python3)
    if(PY3INTERPRETER)
        set(Python3_EXECUTABLE ${PY3INTERPRETER})
        set(Python3_FOUND TRUE)
    endif()
endif()

set(INSTALL_PATH ${CMAKE_INSTALL_DATADIR}/cwipc_util/python)
set(DEST_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_PATH})

install(FILES test_cwipc_util.py setup.py  DESTINATION ${INSTALL_PATH})
install(DIRECTORY cwipc DESTINATION ${INSTALL_PATH} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
if(Python3_FOUND)
	install(CODE "MESSAGE(\"Installing Python modules with ${DEST_INSTALL_PATH}/setup.py\")")
	install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} setup.py install WORKING_DIRECTORY ${DEST_INSTALL_PATH} )")
else()
	MESSAGE("Python3 not found. Install python modules manually from ${INSTALL_PATH}")
endif()


if(Python3_FOUND)
	add_test(NAME run_python_tests
		COMMAND ${Python3_EXECUTABLE} setup.py test
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	
	# Ensure the cwipc_util dynamic library can be found by the tests
	
	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{LD_LIBRARY_PATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		# Jack doesn't understand why modifying PATH doesn't seem to work on Windows...
		# Jack doesn't understand why we need the "Release" here, which should already be in CMAKE_RUNTIME_OUTPUT_DIRECTORY...
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT CWIPC_TEST_DLL=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release/cwipc_util.dll)
	endif()
endif()
