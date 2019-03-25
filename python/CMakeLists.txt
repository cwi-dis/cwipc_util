cmake_minimum_required(VERSION 3.10.0)

set(INSTALL_PATH ${CMAKE_INSTALL_DATADIR}/cwipc_util/python)

install(FILES test_cwipc_util.py setup.py DESTINATION ${INSTALL_PATH})
install(DIRECTORY cwipc DESTINATION ${INSTALL_PATH} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

find_package(Python3 COMPONENTS Interpreter)

# Unfortunately Ubuntu 18.04 doesn't have the Python3 package (introduced in cmake 3.12 or so). Work around it.
if(NOT Python3_FOUND)
    find_program(PY3INTERPRETER python3)
    if(PY3INTERPRETER)
        set(Python3_EXECUTABLE ${PY3INTERPRETER})
        set(Python3_FOUND TRUE)
    endif()
endif()
if(Python3_FOUND)
	add_test(NAME run_python_tests
		COMMAND ${Python3_EXECUTABLE} setup.py test
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	
	# Ensure the cwipc_util dynamic library can be found by the tests
	
	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set_tests_properties(run_python_tests PROPERTIES ENVIRONMENT DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set_tests_properties(run_python_tests PROPERTIES ENVIRONMENT LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{LD_LIBRARY_PATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		# Jack doesn't understand why modifying PATH doesn't seem to work on Windows...
		# Jack doesn't understand why we need the "Release" here, which should already be in CMAKE_RUNTIME_OUTPUT_DIRECTORY...
		set_tests_properties(run_python_tests PROPERTIES ENVIRONMENT CWIPC_TEST_DLL=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release/cwipc_util.dll)
	endif()
endif()