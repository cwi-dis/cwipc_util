#!/bin/sh
# Find Actual INSTALL_PREFIX
bindir=`dirname $0`
installdir=`cd $bindir/.. ; pwd`
sharedir="$installdir/share"
# Determine Python to use: python3 on linux/osx, python on windows, or embeddable python
python3=python3
if ! python3 --version >/dev/null 2>&1 ; then
	python3=python
fi
if [ -f "$installdir/python37embedded/python.exe" ]; then
	python3="$installdir/python37embedded/python.exe"
fi

(cd "$sharedir/cwipc_util/python" ; "$python3" setup.py install)
if [ -d "$sharedir/cwipc_realsense2/python" ]; then
	(cd "$sharedir/cwipc_realsense2/python" ; "$python3" setup.py install)
fi
if [ -d "$sharedir/cwipc_kinect/python" ]; then
	(cd "$sharedir/cwipc_kinect/python" ; "$python3" setup.py install)
fi
(cd "$sharedir/cwipc_codec/python" ; "$python3" setup.py install)
# Finally, if we have installed into an embedded Python, copy phython-based programs into bin
if [ -d "$installdir/bin" -a -d "$installdir/python37embedded/Scripts" ]; then
	ln -f "$installdir/python37embedded/Scripts/cwipc_"* "$installdir/bin/"
fi
