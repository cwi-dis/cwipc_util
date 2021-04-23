cmake_minimum_required(VERSION 3.14.0)
cmake_policy(SET CMP0074 NEW)
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

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	install(PROGRAMS cwipc_pymodules_install_win.bat DESTINATION bin)
endif()
install(PROGRAMS cwipc_pymodules_install.sh DESTINATION bin)
install(PROGRAMS cwipc_tunnelproxy.sh DESTINATION bin)

install(FILES test_cwipc_util.py setup.py  DESTINATION ${INSTALL_PATH})
install(DIRECTORY cwipc DESTINATION ${INSTALL_PATH} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
if(Python3_FOUND)
	install(CODE "MESSAGE(\"Installing Python modules with ${DEST_INSTALL_PATH}/setup.py\")")
	install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} setup.py install WORKING_DIRECTORY ${DEST_INSTALL_PATH} )")
else()
	MESSAGE("Python3 not found. Install python modules manually from ${INSTALL_PATH}")
endif()


if(Python3_FOUND)
	set(ALL_PYTHON_TESTS run_python_tests cwipc_tilecolor cwipc_view cwipc_calibrate)
	#
	# Create venv for testing, and re-set Python3_EXECUTABLE to that Python.
	# But: enable use of system-installed packages, because otherwise CI/CD will take forever.
	#
    get_filename_component(VENV_DIR "${CMAKE_CURRENT_BINARY_DIR}/venv" ABSOLUTE)
    execute_process(COMMAND ${Python3_EXECUTABLE} -m venv --system-site-packages "${VENV_DIR}")
    set(ENV{VIRTUAL_ENV} "${VENV_DIR}")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(Python3_BINDIR "${VENV_DIR}/Scripts")
        set(Python3_EXECUTABLE "${Python3_BINDIR}/python.exe")
    else()
        set(Python3_BINDIR "${VENV_DIR}/bin")
        set(Python3_EXECUTABLE "${Python3_BINDIR}/python")
    endif()
    #
    # Install packages into the venv
    # (later: but scripts into the normal bin directory)
    #
    execute_process(COMMAND ${Python3_EXECUTABLE} -m pip install "${CMAKE_CURRENT_SOURCE_DIR}")
    #
    # Run the unittests
    #
	add_test(NAME run_python_tests
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/setup.py test
	)
	#
	# Test the various scripts
	#
	add_test(NAME cwipc_tilecolor
		COMMAND ${Python3_BINDIR}/cwipc_tilecolor -i ${CMAKE_SOURCE_DIR}/tests/fixtures/input/pcl_frame1.ply -o ${CMAKE_CURRENT_BINARY_DIR}/cwipc_tilecolor_out.ply
	)
	add_test(NAME cwipc_view
		COMMAND ${Python3_BINDIR}/cwipc_view --nodisplay --synthetic --count 10
	)
	add_test(NAME cwipc_calibrate
		COMMAND ${Python3_BINDIR}/cwipc_calibrate --help
	)
	#
	# Ensure the cwipc_util dynamic library can be found by the tests
	#
	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set_property(
			TEST ${ALL_PYTHON_TESTS}
			APPEND PROPERTY ENVIRONMENT "DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH}"
		)
 	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set_property(
			TEST ${ALL_PYTHON_TESTS}
			APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{LD_LIBRARY_PATH}"
		)
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		set_property(
			TEST ${ALL_PYTHON_TESTS}
			APPEND PROPERTY ENVIRONMENT "PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}\;${CMAKE_RUNTIME_OUTPUT_DIRECTORY}\;%PATH%"
		)
	endif()
endif()
