cmake_minimum_required(VERSION 3.16.0)
cmake_policy(SET CMP0074 NEW)

# Setup venv-based Python

include(CwipcPythonSupport)

#
# It is expected that CMAKE_PYWHEELS_INSTALL_DIRECTORY and CMAKE_PYWHEELS_OUTPUT_DIRECTORY
# have been set already.
#

#
# Building.
#

# Create Python wheel in the staging area

cwipc_build_wheel(NAME "cwipc_util" SOURCEDIR ${CMAKE_CURRENT_SOURCE_DIR} WHEELDIR ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})
file(COPY test_cwipc_util.py DESTINATION ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})

#
# Installation.
#

# Copy the whole Python staging subdirectory to the installation (so all scripts are available for inspection and testing)
# Also install the wheel into the current system Python.

install(FILES test_cwipc_util.py DESTINATION ${CMAKE_PYWHEELS_INSTALL_DIRECTORY})
install(DIRECTORY examples DESTINATION  ${CMAKE_PYWHEELS_INSTALL_DIRECTORY})
cwipc_install_wheel(NAME "cwipc_util" WHEELDIR ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})

# Install shell and bat scripts

install(PROGRAMS cwipc_pymodules_install.sh DESTINATION bin)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	install(PROGRAMS cwipc_pymodules_install.bat DESTINATION bin)
	install(PROGRAMS cwipc_pymodules_install.ps1 DESTINATION bin)
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	install(PROGRAMS cwipc_osx_clearquarantine.sh DESTINATION bin)
endif()
install(PROGRAMS cwipc_tunnelproxy.sh DESTINATION bin)

#
# Testing.
#

# setup test environment (installs module into venv)

add_test(NAME _python_setup_cwipc_util
	COMMAND ${Python3_EXECUTABLE} -m pip install --no-cache-dir --upgrade --find-links=${CMAKE_PYWHEELS_OUTPUT_DIRECTORY} cwipc_util
	)
set_tests_properties(_python_setup_cwipc_util PROPERTIES FIXTURES_SETUP venv RUN_SERIAL TRUE)

# Run the unittests

add_test(NAME cwipc_util_python_tests
	COMMAND ${Python3_EXECUTABLE} ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY}/test_cwipc_util.py
	WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
	)
cwipc_python_test(TEST cwipc_util_python_tests FIXTURES_REQUIRED venv)

# Test the various scripts


add_test(NAME cwipc_view
	COMMAND ${Python3_BINDIR}/cwipc_view --nodisplay --synthetic --count 2
	)
cwipc_python_test(TEST cwipc_view FIXTURES_REQUIRED venv)

add_test(NAME cwipc_register
	COMMAND ${Python3_BINDIR}/cwipc_register --help
	)
cwipc_python_test(TEST cwipc_register FIXTURES_REQUIRED venv)
set_tests_properties(cwipc_register PROPERTIES RUN_SERIAL TRUE)

add_test(NAME cwipc_forward
	COMMAND ${Python3_BINDIR}/cwipc_forward --help
	)
cwipc_python_test(TEST cwipc_forward FIXTURES_REQUIRED venv)

add_test(NAME cwipc_toproxy
	COMMAND ${Python3_BINDIR}/cwipc_toproxy --help
	)
cwipc_python_test(TEST cwipc_toproxy FIXTURES_REQUIRED venv)

add_test(NAME cwipc_grab
	COMMAND ${Python3_BINDIR}/cwipc_grab --synthetic --count 2 .
	)
cwipc_python_test(TEST cwipc_grab FIXTURES_REQUIRED venv)

