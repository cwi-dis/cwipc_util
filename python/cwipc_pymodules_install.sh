#!/bin/bash
# Find Actual INSTALL_PREFIX
bindir=`dirname $0`
bindir=`cd $bindir ; pwd`
installdir=`cd $bindir/.. ; pwd`
sharedir="$installdir/share"
# Determine Python to use: cwipc_python on brew, python3 on linux/osx, python on windows
myPython=${CWIPC_PYTHON:=cwipc_python}
if ! $myPython --version >/dev/null 2>&1 ; then
	myPython=python3
fi
if ! $myPython --version >/dev/null 2>&1 ; then
	myPython=python
fi
if ! $myPython --version >/dev/null 2>&1 ; then
	echo "$0: Cannot find suitable Python" >&2
	exit 1
fi
set -x
"$myPython" -m pip --quiet install importlib.metadata
"$myPython" -m pip --quiet uninstall --yes cwipc_util cwipc_codec cwipc_realsense2 cwipc_kinect
"$myPython" -m pip --quiet install --upgrade --find-links="$sharedir/cwipc/python" cwipc_util cwipc_codec
"$myPython" -m pip --quiet install --upgrade --find-links="$sharedir/cwipc/python" cwipc_realsense2
"$myPython" -m pip --quiet install --upgrade --find-links="$sharedir/cwipc/python" cwipc_kinect
